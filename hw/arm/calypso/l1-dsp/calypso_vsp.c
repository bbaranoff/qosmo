/* calypso_vsp.c - port serie voix (VSP) du DSP Calypso.
 *
 * [2026-10-15] LE VOCODEUR DE LA ROM N'AVAIT PAS DE PORT. La ROM 3606 contient
 * tout le chemin voix : decodeur DL (tache 0x7355, codec par le repartiteur
 * *(0x3f64) code 43), codeur UL (tache 0x7400, code 34), et l'ISR par
 * echantillon 0xc14a (vecteur 20, IMR bit 4) qui lit DRR (data 0x20, mvkd 0x20)
 * et ecrit DXR (data 0x21, mvmd bh,0x21) a 8 kHz, avec un FIR de 31 coefficients
 * et les gains d_audio_gain_ul/dl. Sur le Calypso, ce port SP0 est relie a
 * l'ABB (TWL3025, codec voix) par le VSP. Rien de tout cela n'etait modelise :
 * MMR 0x20-0x22 inertes, IT 20 jamais levee -> anneaux 0x3e00 (UL) / 0x4000 (DL)
 * jamais remplis ni vides, et la parole passait par GAPK sur l'hote.
 *
 * Ce modele ne s'accroche pas a data_read/data_write (chemin rapide
 * c54x_rapide) : a chaque echantillon il pose DRR, leve l'IT 20 si la ROM a
 * arme le port (IMR bit 4, SPC = 0xC8 : XRST|RRST), fait tourner l'ISR, puis
 * releve DXR. Debit : 960 echantillons pour 26 trames (8 kHz exacts par rapport
 * au fn), soit 36 ou 37 par trame -- la regulation de la ROM (0xc4f7, ecart
 * d'anneau > 60) saute des trames si le debit derive.
 *
 * Echange avec l'hote : deux anneaux shm (calypso_vsp.h), sans blocage ; un
 * anneau UL vide donne du silence, un DL plein ecrase le plus ancien.
 * pont/vsp_audio.py les relie a PulseAudio (gsm_mic.monitor -> UL, DL ->
 * gsm_audio), les memes points que GAPK et que l'etage « voix » de banc-max.
 *
 * CALYPSO_VSP=1 l'active (defaut 0 : comportement d'avant, GAPK sur l'hote).
 * Le mobile doit alors tourner en `io-handler l1phy` (BANC_VOCODEUR=rom) pour
 * que le firmware leve les mutes du vocodeur (prim_tch.c:101-111). */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/mman.h>
#include "calypso_vsp.h"
#include "c54x_internal.h"
#include "hw/arm/calypso/calypso_debug.h"

typedef struct {
    uint32_t ecrit;
    uint32_t taille;
    int16_t  pcm[VSP_SHM_N];
} VspAnneau;

static VspAnneau *anneau_ouvrir(const char *nom)
{
    int fd = open(nom, O_RDWR | O_CREAT, 0666);
    if (fd < 0) return NULL;
    if (ftruncate(fd, sizeof(VspAnneau)) < 0) { close(fd); return NULL; }
    void *p = mmap(NULL, sizeof(VspAnneau), PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    close(fd);
    if (p == MAP_FAILED) return NULL;
    VspAnneau *a = p;
    a->taille = VSP_SHM_N;
    return a;
}

bool calypso_vsp_actif(void)
{
    static int on = -1;
    if (on < 0) {
        const char *e = calypso_getenv("CALYPSO_VSP");
        on = (e && *e == '1') ? 1 : 0;
        if (on) printf("  [vsp] port voix SP0 modelise (CALYPSO_VSP=1) : DRR 0x20 <- %s, DXR 0x21 -> %s, IT 20\n",
                       VSP_SHM_UL, VSP_SHM_DL);
    }
    return on;
}

static struct {
    VspAnneau *ul, *dl;
    uint32_t   lu_ul;          /* position de lecture dans l'anneau UL */
    unsigned   reste;          /* accumulateur 960/26 */
    unsigned long rint, arme, desarme, sous_ul, trames, saut_ul;
    long       dl_max, ul_max;
    uint32_t   file_ul_max;    /* retard de lecture UL le plus grand de la periode */
} v;

long calypso_vsp_trame(C54xState *dsp, calypso_vsp_courir_fn courir, uint32_t fn)
{
    if (!calypso_vsp_actif() || !dsp || !dsp->running) return 0;
    {   /* CALYPSO_VSP_SHM=0 : pas d'anneaux (rejeu hors ligne, sans toucher au banc) */
        static int shm = -1;
        if (shm < 0) { const char *e = calypso_getenv("CALYPSO_VSP_SHM"); shm = !(e && *e == '0'); }
        if (shm && !v.ul) v.ul = anneau_ouvrir(VSP_SHM_UL);
        if (shm && !v.dl) v.dl = anneau_ouvrir(VSP_SHM_DL);
    }
    v.trames++;
    v.reste += 960;
    unsigned n = v.reste / 26;
    v.reste %= 26;
    long fait = 0;
    for (unsigned i = 0; i < n; i++) {
        uint16_t spc = dsp->data[VSP_SPC];
        bool arme = (dsp->imr & (1u << VSP_IT_BIT)) && (spc & 0xC0) == 0xC0;
        if (!arme) { v.desarme++; continue; }
        v.arme++;
        /* echantillon micro -> DRR */
        int16_t ech = 0;
        if (v.ul) {
            uint32_t ecrit = __atomic_load_n(&v.ul->ecrit, __ATOMIC_ACQUIRE);
            /* [2026-10-16] retard de lecture BORNE. Le rattrapage n'avait lieu qu'au-dela de
             * VSP_SHM_N (2 s) : un DSP en retard au demarrage (camp, ~1000 trames lentes)
             * laissait le micro jusqu'a 2 s derriere, pour toute la duree de la pile (la voix
             * revenait 4,5 s apres le ton, banc du 16/10). CALYPSO_VSP_UL_MAX_MS (defaut 120)
             * borne ce retard : au-dela on repart a la moitie, et on compte le saut. */
            static uint32_t ul_max_ech;
            if (!ul_max_ech) { const char *e = calypso_getenv("CALYPSO_VSP_UL_MAX_MS");
                ul_max_ech = (uint32_t)((e && *e ? atoi(e) : 120) * 8); if (ul_max_ech < 160) ul_max_ech = 160; }
            uint32_t file = ecrit - v.lu_ul;
            if (file > v.file_ul_max) v.file_ul_max = file;
            if (file > ul_max_ech) { v.lu_ul = ecrit - ul_max_ech / 2; v.saut_ul++; }
            if (v.lu_ul != ecrit) ech = v.ul->pcm[v.lu_ul++ % VSP_SHM_N];
            else v.sous_ul++;
            if (abs(ech) > v.ul_max) v.ul_max = abs(ech);
        }
        if (!v.ul) {   /* CALYPSO_VSP_UL_FICHIER=chemin : micro lu dans un fichier s16le (rejeu) */
            static FILE *fu; static int lu;
            if (!lu) { lu = 1; const char *e = calypso_getenv("CALYPSO_VSP_UL_FICHIER"); if (e && *e) fu = fopen(e, "rb"); }
            if (fu && fread(&ech, 2, 1, fu) != 1) { rewind(fu); if (fread(&ech, 2, 1, fu) != 1) ech = 0; }
        }
        dsp->data[VSP_DRR] = (uint16_t)ech;
        c54x_interrupt_ex(dsp, VSP_IT_VEC, VSP_IT_BIT);
        if (dsp->idle && (dsp->ifr & dsp->imr) && !(dsp->st1 & 0x800)) dsp->idle = false;
        uint32_t avant = dsp->insn_count;
        if (!dsp->idle && courir) courir(dsp, 400);
        fait += dsp->insn_count - avant;
        v.rint++;
        /* DXR -> echantillon haut-parleur */
        int16_t sortie = (int16_t)dsp->data[VSP_DXR];
        {   /* CALYPSO_VSP_DL_FICHIER=chemin : copie brute s16le de la sortie (rejeu, analyse hors ligne) */
            static FILE *f; static int lu;
            if (!lu) { lu = 1; const char *e = calypso_getenv("CALYPSO_VSP_DL_FICHIER"); if (e && *e) f = fopen(e, "wb"); }
            if (f) fwrite(&sortie, 2, 1, f);
        }
        if (abs(sortie) > v.dl_max) v.dl_max = abs(sortie);
        if (v.dl) {
            uint32_t w = v.dl->ecrit;
            v.dl->pcm[w % VSP_SHM_N] = sortie;
            __atomic_store_n(&v.dl->ecrit, w + 1, __ATOMIC_RELEASE);
        }
    }
    if (v.trames % 1000 == 0) {
        printf("  [vsp] fn=%u sur 1000 trames : rint=%lu (arme %lu, desarme %lu) sous-alim UL=%lu | crete UL %ld DL %ld | SPC=0x%04x IMR=0x%04x | retard micro max %u ms, sauts %lu\n",
               fn, v.rint, v.arme, v.desarme, v.sous_ul, v.ul_max, v.dl_max, dsp->data[VSP_SPC], dsp->imr,
               v.file_ul_max / 8, v.saut_ul);
        fflush(stdout);
        v.rint = v.arme = v.desarme = v.sous_ul = v.saut_ul = 0; v.dl_max = v.ul_max = 0; v.file_ul_max = 0;
    }
    return fait;
}
