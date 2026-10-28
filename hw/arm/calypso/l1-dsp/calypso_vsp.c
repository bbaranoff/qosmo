/* calypso_vsp.c - voice serial port (VSP) of the Calypso DSP.
 *
 * The 3606 ROM contains the whole voice path: DL decoder (task 0x7355, codec
 * through the dispatcher *(0x3f64) code 43), UL encoder (task 0x7400, code 34),
 * and the per-sample ISR 0xc14a (vector 20, IMR bit 4) which reads DRR (data
 * 0x20, mvkd 0x20) and writes DXR (data 0x21, mvmd bh,0x21) at 8 kHz, with a
 * 31-coefficient FIR and the d_audio_gain_ul/dl gains. On the Calypso this SP0
 * port is wired to the ABB (TWL3025, voice codec) through the VSP. Without this
 * model MMR 0x20-0x22 are inert, IRQ 20 is never raised, the 0x3e00 (UL) /
 * 0x4000 (DL) rings are never filled nor drained, and speech goes through GAPK
 * on the host.
 *
 * This model does not hook data_read/data_write (c54x_rapide fast path): for
 * every sample it sets DRR, raises IRQ 20 if the ROM has armed the port (IMR
 * bit 4, SPC = 0xC8: XRST|RRST), runs the ISR, then collects DXR. Rate: 960
 * samples per 26 frames (exactly 8 kHz relative to the fn), i.e. 36 or 37 per
 * frame -- the ROM's regulation (0xc4f7, ring gap > 60) skips frames if the
 * rate drifts.
 *
 * Exchange with the host: two shm rings (calypso_vsp.h), non-blocking; an
 * empty UL ring yields silence, a full DL ring overwrites the oldest sample.
 * pont/vsp_audio.py connects them to PulseAudio (gsm_mic.monitor -> UL, DL ->
 * gsm_audio), the same points as GAPK and as the "voice" stage of banc-max.
 *
 * CALYPSO_VSP=1 enables it (default 0: GAPK on the host). The mobile must then
 * run with `io-handler l1phy` (BANC_VOCODEUR=rom) so that the firmware lifts
 * the vocoder mutes (prim_tch.c:101-111). */
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
    uint32_t   lu_ul;          /* read position in the UL ring */
    unsigned   reste;          /* 960/26 accumulator */
    unsigned long rint, arme, desarme, sous_ul, trames, saut_ul;
    long       dl_max, ul_max;
    uint32_t   file_ul_max;    /* largest UL read lag over the period */
} v;

long calypso_vsp_trame(C54xState *dsp, calypso_vsp_courir_fn courir, uint32_t fn)
{
    if (!calypso_vsp_actif() || !dsp || !dsp->running) return 0;
    {   /* CALYPSO_VSP_SHM=0: no rings (offline replay, without touching the bench) */
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
        /* microphone sample -> DRR */
        int16_t ech = 0;
        if (v.ul) {
            uint32_t ecrit = __atomic_load_n(&v.ul->ecrit, __ATOMIC_ACQUIRE);
            /* BOUNDED read lag. A DSP running late at startup (camping, ~1000 slow
             * frames) would otherwise leave the microphone up to VSP_SHM_N (2 s) behind
             * for the whole life of the stack. CALYPSO_VSP_UL_MAX_MS (default 120) bounds
             * that lag: beyond it the read position jumps to half of it, and the jump
             * is counted. */
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
        if (!v.ul) {   /* CALYPSO_VSP_UL_FICHIER=path: microphone read from an s16le file (replay) */
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
        /* DXR -> loudspeaker sample */
        int16_t sortie = (int16_t)dsp->data[VSP_DXR];
        {   /* CALYPSO_VSP_DL_FICHIER=path: raw s16le copy of the output (replay, offline analysis) */
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
