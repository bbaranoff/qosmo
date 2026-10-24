/* SPDX-License-Identifier: GPL-2.0-or-later */

#include "qemu/osdep.h"
#include "qemu/timer.h"
#include "qemu/main-loop.h"
#include "exec/cpu-common.h"
#include "hw/core/cpu.h"
#include "target/arm/cpu.h"
#include "hw/arm/calypso/calypso_sim.h"

#define SIM_LOG(...) do { fprintf(stderr, "[sim] " __VA_ARGS__); fputc('\n', stderr); } while(0)
static uint32_t sim_guest_pc(void)
{
    return (first_cpu && first_cpu->cc && first_cpu->cc->get_pc) ? (uint32_t)first_cpu->cc->get_pc(first_cpu) : 0;
}
static uint32_t sim_guest_lr(void)
{
    ARMCPU *cpu = ARM_CPU(first_cpu);
    return (uint32_t)cpu->env.regs[14];
}

#define APDU_MAX_LEN     261
#define RX_FIFO_SIZE     512
#define ATR_DELAY_NS     1000000

static const uint8_t SIM_ATR[] = { 0x3B, 0x02, 0x14, 0x50 };

typedef enum {
    EF_TRANSPARENT = 0,
    EF_LINEAR_FIXED = 1,
    EF_CYCLIC = 2,
    EF_DF = 0xF0,
    EF_MF = 0xF1,
} EfStructure;

typedef struct SimFile {
    uint16_t fid;
    uint16_t parent;
    uint8_t  structure;
    uint16_t size;
    uint8_t  rec_len;
    uint8_t  data[64];
} SimFile;

static SimFile sim_files[] = {
    { 0x3F00, 0x0000, EF_MF, 0, 0, {0} },
    { 0x7F20, 0x3F00, EF_DF, 0, 0, {0} },
    { 0x7F10, 0x3F00, EF_DF, 0, 0, {0} },
    { 0x2FE2, 0x3F00, EF_TRANSPARENT, 10, 0,
      { 0x98, 0x10, 0x10, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0xF1 } },
    { 0x6F07, 0x7F20, EF_TRANSPARENT, 9, 0,
      { 0x08, 0x09, 0x10, 0x10, 0x00, 0x00, 0x00, 0x00, 0xF1 } },
    { 0x6F30, 0x7F20, EF_TRANSPARENT, 24, 0,
      { 0x00, 0xF1, 0x10,
        0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
        0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF } },
    { 0x6F38, 0x7F20, EF_TRANSPARENT, 14, 0,
      { 0xFF, 0x33, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00 } },
    { 0x6F78, 0x7F20, EF_TRANSPARENT, 2, 0,
      { 0x00, 0x01 } },
    { 0x6FAD, 0x7F20, EF_TRANSPARENT, 4, 0,
      { 0x00, 0x00, 0x00, 0x02 } },
    { 0x6F7E, 0x7F20, EF_TRANSPARENT, 11, 0,
      { 0xFF, 0xFF, 0xFF, 0xFF,
        0xFF, 0xFF, 0xFF,
        0x00, 0x00,
        0xFF, 0x00 } },
    /* [2026-10-21] EF_BCCH (liste BA) : seul l'ARFCN 1 (bit 6 de l'octet 0, MSB d'abord).
     * A 0xFF la liste couvrait presque tous les ARFCN : la recherche balayait 1..409 au hasard. */
    { 0x6F74, 0x7F20, EF_TRANSPARENT, 16, 0,
      { 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 } },
    { 0x6F46, 0x7F20, EF_TRANSPARENT, 17, 0,
      { 0x01, 'Q','E','M','U','-','S','I','M', ' ',' ',' ',' ',' ',' ',' ',' ' } },
    /* [2026-10-21] EF absents -> « SIM Invalid » sur Magnetite (la pile TI lit Kc et
     * PHASE a l'activation ; « file not found » les fait echouer). GSM 11.11 :
     * Kc = 8 octets + CKSN (0x07 = pas de cle) ; PHASE 0x03 = phase 2+ ;
     * HPLMN 0xFF = recherche desactivee ; FPLMN vide (0xFF). */
    { 0x6F20, 0x7F20, EF_TRANSPARENT, 9, 0,
      { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07 } },
    /* [2026-10-21] ECC (112) et LP (anglais) : « file not found » juste apres l'activation,
     * le MMI affiche « SIM Invalid » (activation SIM_NO_ERROR, cause 0x80FF mesuree). */
    { 0x6FB7, 0x7F20, EF_TRANSPARENT, 3, 0,
      { 0x11, 0xF2, 0xFF } },
    { 0x6F05, 0x7F20, EF_TRANSPARENT, 1, 0,
      { 0x01 } },
    /* [2026-10-21] EF_ADN (repertoire) : la SST annonce ADN (service 1). Sans ce fichier le driver
     * SIM met crdFun = SIM_NO_OPERATION -> SS_INV -> CME_ERR_SimWrong -> « SIM Invalid ». */
    { 0x6FAE, 0x7F20, EF_TRANSPARENT, 1, 0,
      { 0x03 } },
    { 0x6F31, 0x7F20, EF_TRANSPARENT, 1, 0,
      { 0xFF } },
    { 0x6F7B, 0x7F20, EF_TRANSPARENT, 12, 0,
      { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF } },
    /* [2026-10-21] EF_SMS : capacite de stockage SMS (memory.total = nb d'enregistrements).
     * Sans ce fichier g_max_messages = 0 -> ALLOC_MEMORY(0) dans smsidle_sms_cb -> « mfwAlloc invalid »
     * et « Messages: full ». 4 enregistrements libres de 176 octets. */
    { 0x6F3C, 0x7F20, EF_LINEAR_FIXED, 704, 176,
      { 0 } },
    /* [2026-10-21] SMS init : EF SMSP (1 enregistrement libre), SMSS et CBMI vides */
    { 0x6F42, 0x7F20, EF_LINEAR_FIXED, 28, 28,
      { 0xFF } },
    { 0x6F43, 0x7F20, EF_TRANSPARENT, 2, 0,
      { 0xFF, 0xFF } },
    { 0x6F45, 0x7F20, EF_TRANSPARENT, 2, 0,
      { 0xFF, 0xFF } },
    { 0x6F48, 0x7F20, EF_TRANSPARENT, 2, 0,
      { 0xFF, 0xFF } },
    /* [2026-10-22 14:55] EF que Magnetite demande et qui manquaient (« file not found » dans qemu.log) : apres
     * chaque SMS recu le telephone lit EF_ACM (6F39) ; puis tout le menu SMS (lire, etat memoire) reste sur
     * « Please wait » sans plus aucun echange SIM. Contenus neutres (GSM 11.11) : compteurs a zero, tarif
     * absent, repertoires vides (enregistrements libres = FF). */
    { 0x6F39, 0x7F20, EF_CYCLIC, 9, 3,          /* EF_ACM : 3 enregistrements de 3 octets, 0 */
      { 0 } },
    { 0x6F37, 0x7F20, EF_TRANSPARENT, 3, 0,     /* EF_ACMmax : 0 = pas de limite */
      { 0x00, 0x00, 0x00 } },
    { 0x6F41, 0x7F20, EF_TRANSPARENT, 5, 0,     /* EF_PUCT : devise et prix absents */
      { 0xFF, 0xFF, 0xFF, 0x00, 0x00 } },
    { 0x6F3E, 0x7F20, EF_TRANSPARENT, 1, 0,     /* EF_GID1 */
      { 0xFF } },
    { 0x6F3F, 0x7F20, EF_TRANSPARENT, 1, 0,     /* EF_GID2 */
      { 0xFF } },
    { 0x6F3A, 0x7F10, EF_LINEAR_FIXED, 280, 28, /* EF_ADN : 10 entrees libres */
      { 0xFF } },
    { 0x6F3B, 0x7F10, EF_LINEAR_FIXED, 28, 28,  /* EF_FDN : 1 entree libre */
      { 0xFF } },
    { 0x6F40, 0x7F10, EF_LINEAR_FIXED, 28, 28,  /* EF_MSISDN */
      { 0xFF } },
    { 0x6F44, 0x7F10, EF_CYCLIC, 140, 28,       /* EF_LND : 5 derniers numeros */
      { 0xFF } },
    { 0x6F4A, 0x7F10, EF_LINEAR_FIXED, 13, 13,  /* EF_EXT1 */
      { 0xFF } },
    { 0x6F4B, 0x7F10, EF_LINEAR_FIXED, 13, 13,  /* EF_EXT2 */
      { 0xFF } },
    /* [2026-10-22 15:05] CPHS (firmware construit avec FF_CPHS) : 6F11/6F13/6F14/6F16 « file not found » et le
     * menu Messages (qui porte la messagerie vocale CPHS) reste sur « Please wait ». CPHS info (6F16) : phase 2,
     * aucun service alloue ; indicateurs vides (0x55 = pas de message / pas de renvoi), nom d'operateur absent. */
    { 0x6F16, 0x7F20, EF_TRANSPARENT, 3, 0,     /* CPHS info : phase 2, services 00 00 */
      { 0x02, 0x00, 0x00 } },
    { 0x6F11, 0x7F20, EF_TRANSPARENT, 2, 0,     /* CPHS voice message waiting */
      { 0x55, 0x55 } },
    { 0x6F13, 0x7F20, EF_TRANSPARENT, 2, 0,     /* CPHS call forward flags */
      { 0x55, 0x55 } },
    { 0x6F14, 0x7F20, EF_TRANSPARENT, 10, 0,    /* CPHS operator name string */
      { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF } },
    { 0x6F15, 0x7F20, EF_TRANSPARENT, 2, 0,     /* CPHS customer service profile */
      { 0xFF, 0xFF } },
    { 0x6F17, 0x7F20, EF_LINEAR_FIXED, 28, 28,  /* CPHS mailbox numbers : 1 entree libre */
      { 0xFF } },
};
#define SIM_FILE_COUNT (sizeof(sim_files) / sizeof(sim_files[0]))

static SimFile *find_file(uint16_t fid, uint16_t parent_pref)
{

    if (parent_pref) {
        for (size_t i = 0; i < SIM_FILE_COUNT; i++)
            if (sim_files[i].fid == fid && sim_files[i].parent == parent_pref)
                return &sim_files[i];
    }
    for (size_t i = 0; i < SIM_FILE_COUNT; i++)
        if (sim_files[i].fid == fid)
            return &sim_files[i];
    return NULL;
}

static int encode_imsi_bcd(const char *imsi_str, uint8_t out[9])
{
    int n = 0;
    while (imsi_str[n] >= '0' && imsi_str[n] <= '9') n++;
    if (n < 1 || n > 15) return -1;
    bool odd = (n & 1) != 0;
    int data_len = (n / 2) + 1;
    out[0] = (uint8_t)data_len;
    out[1] = (uint8_t)(((imsi_str[0] - '0') << 4) | (odd ? 0x9 : 0x1));
    int wpos = 2, ipos = 1;
    while (ipos < n) {
        uint8_t lo = imsi_str[ipos] - '0';
        uint8_t hi = (ipos + 1 < n) ? (imsi_str[ipos + 1] - '0') : 0xF;
        out[wpos++] = (uint8_t)((hi << 4) | lo);
        ipos += 2;
    }
    while (wpos < 9) out[wpos++] = 0xFF;
    return data_len + 1;
}

struct CalypsoSim {
    qemu_irq    irq;
    QEMUTimer  *atr_timer;
    QEMUTimer  *wt_timer;

    uint8_t     ki[16];
    bool        ki_valid;

    uint16_t    cmd, stat, conf1, conf2, maskit, it_cd;

    uint16_t    it;

    uint8_t     apdu[APDU_MAX_LEN];
    int         apdu_pos;
    int         apdu_expected;

    uint8_t     rx[RX_FIFO_SIZE];
    int         rx_head, rx_tail;

    uint16_t    selected_df;
    uint16_t    selected_ef;

    uint8_t     resp_buf[64];
    int         resp_len;

    bool        powered;
    bool        atr_manual_done;   /* [2026-10-21] ATR deja livre pour ce reset (mode manuel TI) */
    /* [2026-10-21] reponses de la carte differees d'un temps de transmission
     * (le driver TI attend l'octet de procedure APRES avoir fini d'emettre et
     * bascule SIO en reception ; une reponse synchrone dans l'ecriture de DTX
     * arrivait trop tot et il recommencait la commande) */
    bool        defer;
    uint8_t     pend[APDU_MAX_LEN + 16];
    int         pend_len;
    QEMUTimer  *resp_timer;
    /* [2026-10-24] IT TX apres un temps caractere (voir calypso_sim_reg_write, DTX) */
    QEMUTimer  *tx_timer;
    int64_t     last_tx_ns;
};

G_GNUC_UNUSED
static int rx_count(CalypsoSim *s)
{
    int c = s->rx_head - s->rx_tail;
    if (c < 0) c += RX_FIFO_SIZE;
    return c;
}

static void rx_push(CalypsoSim *s, uint8_t b)
{
    if (s->defer) {
        if (s->pend_len < (int)sizeof(s->pend)) {
            s->pend[s->pend_len++] = b;
        }
        return;
    }
    int next = (s->rx_head + 1) % RX_FIFO_SIZE;
    if (next == s->rx_tail) {
        SIM_LOG("RX FIFO overflow");
        return;
    }
    s->rx[s->rx_head] = b;
    s->rx_head = next;
}

G_GNUC_UNUSED
static int rx_pop(CalypsoSim *s, uint8_t *out)
{
    if (s->rx_head == s->rx_tail) return 0;
    *out = s->rx[s->rx_tail];
    s->rx_tail = (s->rx_tail + 1) % RX_FIFO_SIZE;
    return 1;
}

static void rx_push_n(CalypsoSim *s, const uint8_t *buf, int n)
{
    for (int i = 0; i < n; i++) rx_push(s, buf[i]);
}

static void update_irq(CalypsoSim *s);

#define WT_DELAY_NS  2000000

static void fire_wt(void *opaque)
{
    CalypsoSim *s = opaque;
    if (rx_count(s) > 0) return;
    /* [2026-10-22 15:15] reponse en cours d'emission (delai proportionnel, resp_delai_ns) : la carte « parle »,
     * le temps d'attente est relance a chaque octet sur une vraie carte -> on ne declare pas de WT. */
    if (s->pend_len > 0) { timer_mod_ns(s->wt_timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + WT_DELAY_NS); return; }
    s->it |= CALYPSO_SIM_IT_WT;
    SIM_LOG("WT timeout fired (RX FIFO empty)");
    update_irq(s);

}

static void schedule_wt(CalypsoSim *s)
{
    if (s->wt_timer)
        timer_mod_ns(s->wt_timer,
                     qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + WT_DELAY_NS);
}

static void refresh_it_rx(CalypsoSim *s)
{
    if (rx_count(s) > 0) s->it |= CALYPSO_SIM_IT_RX;
    else                 s->it &= ~CALYPSO_SIM_IT_RX;
}

static void update_irq(CalypsoSim *s)
{
    if (!s->irq) return;
    refresh_it_rx(s);
    bool active = (s->it & ~(uint16_t)s->maskit) != 0;
    static unsigned log_count;
    static bool last_active;
    if ((active != last_active && log_count < 30) || (log_count < 10)) {
        SIM_LOG("IRQ %s  IT=0x%04x MASKIT=0x%04x rx_count=%d",
                active ? "RAISE" : "lower",
                s->it, s->maskit, rx_count(s));
        log_count++;
        last_active = active;
    }
    if (active) qemu_irq_raise(s->irq);
    else        qemu_irq_lower(s->irq);
}

#define RESP_DELAY_NS 1500000   /* ~1,5 ms : un octet a 9600 bauds + marge */
/* [2026-10-22 15:15] DELAI PROPORTIONNEL A LA REPONSE. 1,5 ms par reponse quelle que soit sa taille : toute
 * l'initialisation SMS de Magnetite (4 x 176 octets d'EF_SMS, SMSP, CBMI) finissait avant que l'ecran de veille
 * ne s'abonne a E_SMS_READY (smsidle_start, fin de mise sous tension) : evenement perdu, sms_initialised reste
 * FALSE, le menu Messages reste sur « Please wait » (SmsRead_R_exec_cb). Une vraie carte a 9600 bauds rend
 * ~1,04 ms par octet. CALYPSO_SIM_US_OCTET (defaut 1040, 0 = ancien delai fixe). */
static long us_octet(void)
{
    static long us = -1;
    if (us < 0) { const char *e = getenv("CALYPSO_SIM_US_OCTET"); us = (e && *e) ? atol(e) : 1040; }
    return us;
}

static int64_t resp_delai_ns(int n)
{
    int64_t d = (int64_t)n * us_octet() * 1000;
    return d > RESP_DELAY_NS ? d : RESP_DELAY_NS;
}

static void tx_fire(void *opaque)
{
    CalypsoSim *s = opaque;
    s->it |= CALYPSO_SIM_IT_TX;
    update_irq(s);
}

static void resp_fire(void *opaque)
{
    CalypsoSim *s = opaque;
    int i;
    bool was = s->defer;
    s->defer = false;
    for (i = 0; i < s->pend_len; i++) {
        rx_push(s, s->pend[i]);
    }
    if (s->pend_len) {
        SIM_LOG("reponse carte : %d octet(s) livres", s->pend_len);
    }
    s->pend_len = 0;
    s->defer = was && false;
    update_irq(s);
}

static void raise_rx_irq(CalypsoSim *s)
{
    if (s->defer) {
        timer_mod_ns(s->resp_timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + resp_delai_ns(s->pend_len));
        return;
    }
    update_irq(s);
}

static void deliver_atr(void *opaque)
{
    CalypsoSim *s = opaque;
    if (!s->powered) return;
    rx_push_n(s, SIM_ATR, sizeof(SIM_ATR));
    SIM_LOG("ATR queued (%zu bytes)", sizeof(SIM_ATR));
    raise_rx_irq(s);
}

G_GNUC_UNUSED
static void schedule_atr(CalypsoSim *s)
{
    timer_mod_ns(s->atr_timer,
                 qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + ATR_DELAY_NS);
}

static int build_select_response(CalypsoSim *s, SimFile *f, uint8_t *out)
{

    int p = 0;
    out[p++] = 0x00; out[p++] = 0x00;
    out[p++] = (f->size >> 8) & 0xFF;
    out[p++] =  f->size       & 0xFF;
    out[p++] = (f->fid >> 8) & 0xFF;
    out[p++] =  f->fid       & 0xFF;
    if (f->structure == EF_DF || f->structure == EF_MF) {
        /* [2026-10-21] GSM 11.11 9.2.1, reponse a SELECT d'un MF/DF :
         *  6 type (01 MF, 02 DF) ; 7-12 RFU ; 13 caracteristiques du fichier ;
         *  14 nb de DF ; 15 nb d'EF ; 16 nb de CHV ; 17 RFU ; 18 etat CHV1 ;
         *  19 UNBLOCK CHV1 ; 20 CHV2 ; 21 UNBLOCK CHV2.
         * L'octet 13 valait 0 : « carte 5 V seulement » (b5 = 0) -> le driver
         * SIM de TI (sim.c, test 27.17.1.5.2) coupait la carte et la rejetait.
         * 0x11 = arret d'horloge permis, technologie 3 V. CHV1 initialise
         * (0x83), 3 essais, de meme pour CHV2 et les codes de deblocage. */
        out[p++] = (f->structure == EF_MF) ? 0x01 : 0x02;   /* 6 */
        out[p++] = 0x00; out[p++] = 0x00; out[p++] = 0x00;  /* 7-9 */
        out[p++] = 0x00; out[p++] = 0x00; out[p++] = 0x00;  /* 10-12 */
        out[p++] = 0x91;                                    /* 13 : + b8 CHV1 desactive (pas de « Enter PIN1 ») */
        out[p++] = 0x02;                                    /* 14 DFs */
        out[p++] = 0x09;                                    /* 15 EFs */
        out[p++] = 0x04;                                    /* 16 CHVs */
        out[p++] = 0x00;                                    /* 17 */
        out[p++] = 0x83; out[p++] = 0x8A;                   /* 18-19 */
        out[p++] = 0x83; out[p++] = 0x8A;                   /* 20-21 */
        out[p++] = 0x00; out[p++] = 0x00; out[p++] = 0x00;  /* 22-24 */
    } else {
        out[p++] = 0x04;
        out[p++] = 0x00;
        out[p++] = 0x00; out[p++] = 0x00;
        out[p++] = 0x00;
        /* [2026-10-21] file status (octet 11) : b1 = 1 -> fichier non invalide. 0xAA (b1 = 0)
         * faisait croire a app_check_imsi_loci_validation() que IMSI/LOCI sont invalides ->
         * rehabilitation -> SIM_NO_OPERATION (« SIM Invalid ») */
        out[p++] = 0xAB;
        /* [2026-10-21] octets 13-15 (GSM 11.11 9.2.1) : longueur des donnees suivantes (2), structure,
         * longueur d'enregistrement. Trois zeros de trop decalaient tout : structure lue 00
         * (transparent) et longueur d'enregistrement 0 -> l'entite SIM ne lisait que l'enregistrement 1
         * de EF SMS, capacite SMS 0 -> mfwAlloc(0) en boucle et « Messages: full » dans l'IHM. */
        out[p++] = 0x02;
        out[p++] = (f->structure == EF_TRANSPARENT) ? 0x00
                : (f->structure == EF_LINEAR_FIXED) ? 0x01 : 0x03;
        out[p++] = f->rec_len;
    }
    return p;
}

static void respond_sw(CalypsoSim *s, uint8_t sw1, uint8_t sw2)
{
    rx_push(s, sw1);
    rx_push(s, sw2);
}

static void respond_with_data_pending(CalypsoSim *s,
                                       const uint8_t *data, int len)
{
    if (len > (int)sizeof(s->resp_buf)) len = sizeof(s->resp_buf);
    memcpy(s->resp_buf, data, len);
    s->resp_len = len;

    respond_sw(s, 0x9F, (uint8_t)len);
}

static void cmd_select(CalypsoSim *s, uint8_t p1, uint8_t p2,
                       uint8_t lc, const uint8_t *data)
{
    if (lc != 2) { respond_sw(s, 0x67, 0x00); return; }
    uint16_t fid = (data[0] << 8) | data[1];

    SimFile *f = NULL;
    if (fid == 0x3F00) {
        f = find_file(0x3F00, 0);
        s->selected_df = 0x3F00;
    } else if ((fid & 0xFF00) == 0x7F00) {
        f = find_file(fid, 0);
        if (f) s->selected_df = fid;
    } else if ((fid & 0xFF00) == 0x2F00) {
        f = find_file(fid, 0x3F00);
    } else {
        f = find_file(fid, s->selected_df);
        if (!f) f = find_file(fid, 0);
    }

    if (!f) {
        SIM_LOG("SELECT 0x%04x → file not found", fid);
        respond_sw(s, 0x6A, 0x82);
        return;
    }
    s->selected_ef = f->fid;

    uint8_t resp[32];
    int n = build_select_response(s, f, resp);
    SIM_LOG("pc=0x%08x SELECT 0x%04x (%s) → %d bytes pending", sim_guest_pc(),
            fid,
            f->structure == EF_MF ? "MF" :
            f->structure == EF_DF ? "DF" : "EF",
            n);
    respond_with_data_pending(s, resp, n);
}

static void cmd_get_response(CalypsoSim *s, uint8_t le)
{
    if (s->resp_len == 0) {
        respond_sw(s, 0x6F, 0x00);
        return;
    }
    int n = (le == 0) ? s->resp_len : (le > s->resp_len ? s->resp_len : le);
    rx_push_n(s, s->resp_buf, n);
    respond_sw(s, 0x90, 0x00);
    s->resp_len = 0;
}

/* [2026-10-22 14:35] ENREGISTREMENTS GARDES. Les lectures rendaient 0xFF partout, EF_SMS compris : un enregistrement
 * SMS libre commence par l'octet d'etat 0x00 (GSM 11.11 10.5.3), 0xFF n'en est pas un. La pile SMS de Magnetite ne
 * trouvait donc aucune place (« [cmhSMS_checkAccess]: Wrong SIM status », « Messages: full ») et repondait RP-ERROR
 * a chaque SMS entrant (MSC : « Sending SMS ... failed »). Et UPDATE RECORD n'etait pas garde. Un tampon par fichier
 * a enregistrements, initialise libre (EF_SMS : 00 puis FF ; les autres : FF), lu et ecrit par record. */
#define SIM_REC_MAX 1024
/* [2026-10-22] CENTRE SMS. EF_SMSP restait libre (FF) : en SMS MO le C139 demandait « Cent. » a la main et le
 * firmware a fini dans dar_reset (boucle de resets du chien de garde). On pose le centre du banc dans
 * l'enregistrement 1 (GSM 11.11 10.5.6) : indicateurs E1 = adresse SC, PID, DCS et VP presents, TP-DA absent.
 * Le numero vient de « sms-service-center » de CALYPSO_SIM_CFG (mobile.cfg), sinon celui du banc. */
static char g_sim_smsc[24] = "+3366612340001";
static void smsp_remplir(uint8_t *r)          /* r : enregistrement de 28 octets, deja a FF */
{
    const char *d = g_sim_smsc;
    uint8_t ton = 0x81;
    if (*d == '+') { ton = 0x91; d++; }
    int n = 0;
    uint8_t bcd[10];
    memset(bcd, 0xFF, sizeof bcd);
    for (; *d && n < 20; d++) {
        if (*d < '0' || *d > '9') continue;
        uint8_t v = (uint8_t)(*d - '0');
        bcd[n / 2] = (n & 1) ? (uint8_t)((bcd[n / 2] & 0x0F) | (v << 4)) : (uint8_t)(0xF0 | v);
        n++;
    }
    if (!n) return;
    r[0] = 0xE1;
    r[13] = (uint8_t)(1 + (n + 1) / 2);       /* longueur : TON/NPI + chiffres */
    r[14] = ton;
    memcpy(&r[15], bcd, (n + 1) / 2);
    r[25] = 0x00;                             /* TP-PID */
    r[26] = 0x00;                             /* TP-DCS */
    r[27] = 0xA7;                             /* TP-VP : 24 h */
}
static uint8_t *rec_store(SimFile *f)
{
    static uint8_t *store[SIM_FILE_COUNT];
    size_t i = (size_t)(f - sim_files);
    if (i >= SIM_FILE_COUNT || f->rec_len == 0 || f->size > SIM_REC_MAX) return NULL;
    if (!store[i]) {
        store[i] = g_malloc(f->size);
        memset(store[i], 0xFF, f->size);
        if (f->fid == 0x6F3C)
            for (int r = 0; r < f->size / f->rec_len; r++) store[i][r * f->rec_len] = 0x00;
        if (f->fid == 0x6F39) memset(store[i], 0x00, f->size);   /* EF_ACM : compteur a zero */
        if (f->fid == 0x6F42 && f->rec_len == 28) smsp_remplir(store[i]);   /* EF_SMSP : centre SMS */
    }
    return store[i];
}

static void cmd_read_record(CalypsoSim *s, uint8_t rec, uint8_t mode, uint8_t le)
{
    SimFile *f = find_file(s->selected_ef, 0);
    if (!f || (f->structure != EF_LINEAR_FIXED && f->structure != EF_CYCLIC)) {
        respond_sw(s, 0x94, 0x00);
        return;
    }
    int nrec = f->size / f->rec_len;
    /* [2026-10-21] modes 02 (suivant) / 03 (precedent) en plus de 04 (absolu), GSM 11.11 9.2.5 */
    static uint16_t rec_ef; static int rec_cur;
    if (rec_ef != s->selected_ef) { rec_ef = s->selected_ef; rec_cur = 0; }
    if (mode == 0x02) rec = (uint8_t)(rec_cur + 1);
    else if (mode == 0x03) rec = (uint8_t)(rec_cur > 0 ? rec_cur - 1 : nrec);
    else if (mode != 0x04) { respond_sw(s, 0x6B, 0x00); return; }
    if (rec < 1 || rec > nrec) {
        respond_sw(s, 0x94, 0x02);
        return;
    }
    rec_cur = rec;
    /* enregistrements libres (0xFF) : pas de contenu stocke pour ce fichier */
    uint8_t buf[256];
    int n = (le == 0) ? f->rec_len : le;
    if (n > f->rec_len) n = f->rec_len;
    memset(buf, 0xFF, n);
    { uint8_t *st = rec_store(f); if (st) memcpy(buf, st + (rec - 1) * f->rec_len, n); }
    rx_push_n(s, buf, n);
    SIM_LOG("READ_RECORD EF=0x%04x rec=%d len=%d : %02x %02x %02x %02x %02x %02x %02x %02x ...", s->selected_ef, rec, n,
            buf[0], buf[1], buf[2], buf[3], buf[4], buf[5], buf[6], buf[7]);
    respond_sw(s, 0x90, 0x00);
}

static void cmd_read_binary(CalypsoSim *s, uint8_t p1, uint8_t p2, uint8_t le)
{
    SimFile *f = find_file(s->selected_ef, 0);
    if (!f || f->structure != EF_TRANSPARENT) {
        respond_sw(s, 0x94, 0x00);
        return;
    }
    int offset = (p1 << 8) | p2;
    int n = (le == 0) ? 256 : le;
    if (offset + n > f->size) {
        respond_sw(s, 0x67, 0x00);
        return;
    }
    rx_push_n(s, f->data + offset, n);
    SIM_LOG("READ_BINARY EF=0x%04x off=%d len=%d", s->selected_ef, offset, n);
    respond_sw(s, 0x90, 0x00);
}

/* [2026-10-21] UPDATE BINARY / UPDATE RECORD : refuses (6D00) jusqu'ici, le telephone ne pouvait
 * rien ecrire (LOCI, Kc, FPLMN, SMS...). Le transparent est garde dans data[] tant qu'il y tient ;
 * les enregistrements sont acceptes sans etre gardes (les lectures rendent des enregistrements libres). */
static void cmd_update_binary(CalypsoSim *s, uint8_t p1, uint8_t p2, uint8_t lc, const uint8_t *data)
{
    SimFile *f = find_file(s->selected_ef, 0);
    if (!f || f->structure != EF_TRANSPARENT) { respond_sw(s, 0x94, 0x08); return; }
    int offset = (p1 << 8) | p2;
    if (offset + lc > f->size) { respond_sw(s, 0x67, 0x00); return; }
    if (offset + lc <= (int)sizeof f->data) memcpy(f->data + offset, data, lc);
    SIM_LOG("UPDATE_BINARY EF=0x%04x off=%d len=%d", s->selected_ef, offset, lc);
    respond_sw(s, 0x90, 0x00);
}

static void cmd_update_record(CalypsoSim *s, uint8_t rec, uint8_t mode, uint8_t lc, const uint8_t *data)
{
    SimFile *f = find_file(s->selected_ef, 0);
    if (!f || (f->structure != EF_LINEAR_FIXED && f->structure != EF_CYCLIC)) { respond_sw(s, 0x94, 0x08); return; }
    if (lc != f->rec_len) { respond_sw(s, 0x67, 0x00); return; }
    int nrec = f->size / f->rec_len;
    if (mode == 0x04 && (rec < 1 || rec > nrec)) { respond_sw(s, 0x94, 0x02); return; }
    if (mode == 0x04) { uint8_t *st = rec_store(f); if (st && data) memcpy(st + (rec - 1) * f->rec_len, data, lc); }
    SIM_LOG("UPDATE_RECORD EF=0x%04x rec=%d mode=%d len=%d : %02x %02x %02x %02x %02x %02x %02x %02x ...", s->selected_ef, rec, mode, lc,
            data ? data[0] : 0, data ? data[1] : 0, data ? data[2] : 0, data ? data[3] : 0,
            data ? data[4] : 0, data ? data[5] : 0, data ? data[6] : 0, data ? data[7] : 0);
    respond_sw(s, 0x90, 0x00);
}

static void cmd_status(CalypsoSim *s, uint8_t le)
{
    SimFile *df = find_file(s->selected_df, 0);
    if (!df) { respond_sw(s, 0x6F, 0x00); return; }
    uint8_t resp[32];
    int n = build_select_response(s, df, resp);
    int outn = (le == 0 || le > n) ? n : le;
    rx_push_n(s, resp, outn);
    respond_sw(s, 0x90, 0x00);
}

static void cmd_verify_chv(CalypsoSim *s, uint8_t p2, uint8_t lc,
                           const uint8_t *data)
{

    (void)p2; (void)lc; (void)data;
    SIM_LOG("VERIFY_CHV → always OK");
    respond_sw(s, 0x90, 0x00);
}

void comp128v1(const uint8_t *ki, const uint8_t *rand, uint8_t *sres, uint8_t *kc);   /* calypso_comp128.c */
static void cmd_run_gsm_algo(CalypsoSim *s, uint8_t lc, const uint8_t *data)
{

    if (lc != 16) { respond_sw(s, 0x67, 0x00); return; }

    uint8_t resp[12];
    /* [2026-10-21] le VRAI calcul : COMP128v1 avec la Ki de mobile.cfg (calypso_comp128.c). Le faux SRES
     * (RAND ^ 0xAA, Kc 0) ne genait pas le mobile osmocom (sim test : comp128 fait dans layer23), mais le
     * firmware TI (Magnetite) demande le calcul a la carte -> LOCATION UPDATING REJECT du MSC. */
    if (s->ki_valid) {
        comp128v1(s->ki, data, resp, resp + 4);
        SIM_LOG("RUN_GSM_ALGORITHM COMP128v1 (RAND[0]=0x%02x) → SRES=%02x%02x%02x%02x Kc[0]=0x%02x",
                data[0], resp[0], resp[1], resp[2], resp[3], resp[4]);
    } else {
        for (int i = 0; i < 4; i++) resp[i]    = data[i] ^ 0xAA;
        for (int i = 0; i < 8; i++) resp[4+i] = 0x00;
        SIM_LOG("RUN_GSM_ALGORITHM SANS Ki (CALYPSO_SIM_CFG ?) → SRES bidon RAND^0xAA");
    }
    respond_with_data_pending(s, resp, 12);
}

static void dispatch_apdu(CalypsoSim *s)
{
    uint8_t cla = s->apdu[0];
    uint8_t ins = s->apdu[1];
    uint8_t p1  = s->apdu[2];
    uint8_t p2  = s->apdu[3];
    uint8_t p3  = s->apdu[4];
    uint8_t *data = (s->apdu_pos > 5) ? s->apdu + 5 : NULL;
    int     dlen = s->apdu_pos - 5;

    SIM_LOG("APDU CLA=%02x INS=%02x P1=%02x P2=%02x P3=%02x dlen=%d",
            cla, ins, p1, p2, p3, dlen > 0 ? dlen : 0);

    /* [2026-10-21] T=0 : une commande qui RENVOIE des donnees (GET RESPONSE,
     * READ BINARY/RECORD, STATUS, FETCH) commence par l'octet de procedure
     * (ACK = INS) avant les donnees puis SW1 SW2. Le driver TI (sim32.c,
     * moderx 1) l'exige ; sans lui le premier octet de donnees passait pour un
     * ACK invalide et SIM_GetResponse echouait. Les commandes sortantes ont deja
     * recu leur ACK apres l'en-tete (apdu_tx_byte), les commandes sans donnees
     * (moderx 5) attendent directement SW1 SW2. */
    if (ins == 0xC0 || ins == 0xB0 || ins == 0xB2 || ins == 0xF2 || ins == 0x12) {
        rx_push(s, ins);
    }

    switch (ins) {
    case 0xA4: cmd_select(s, p1, p2, p3, data ? data : (const uint8_t *)""); break;
    case 0xB0: cmd_read_binary(s, p1, p2, p3); break;
    case 0xB2: cmd_read_record(s, p1, p2, p3); break;
    case 0xD6: cmd_update_binary(s, p1, p2, p3, data ? data : (const uint8_t *)""); break;
    case 0xDC: cmd_update_record(s, p1, p2, p3, data); break;
    case 0xC0: cmd_get_response(s, p3); break;
    case 0xF2: cmd_status(s, p3); break;
    case 0x20: cmd_verify_chv(s, p2, p3, data ? data : (const uint8_t *)""); break;
    case 0x88: cmd_run_gsm_algo(s, p3, data ? data : (const uint8_t *)""); break;
    default:
        SIM_LOG("INS=0x%02x not supported → SW=6D00", ins);
        respond_sw(s, 0x6D, 0x00);
        break;
    }

    raise_rx_irq(s);
}

G_GNUC_UNUSED
static void apdu_tx_byte_inner(CalypsoSim *s, uint8_t b);
static void apdu_tx_byte(CalypsoSim *s, uint8_t b)
{
    s->defer = true;
    apdu_tx_byte_inner(s, b);
    s->defer = false;
    if (s->pend_len) {
        timer_mod_ns(s->resp_timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + resp_delai_ns(s->pend_len));
    }
}

/* [2026-10-24] RESYNCHRONISATION. Si le driver TI n'a pas vu l'ACK (SIM_ERR_WAIT au bout de 159 a 1815 ticks,
 * soit 0,7 s au moins), la couche au-dessus renvoie un en-tete neuf alors qu'on attendait la suite de la commande :
 * « a0 a4 » etait pris pour le FID (SELECT 0xa0a4) et toutes les commandes suivantes restaient decalees de 2 octets
 * (CLA=00 INS=00 -> 6D00 a perpetuite -> « SIM Invalid »). Dans une commande les octets se suivent a ~1 ms : un
 * silence de plus de 500 ms veut dire que le terminal a abandonne la commande, l'octet recu ouvre un nouvel APDU. */
#define APDU_ABANDON_NS 500000000LL
static void apdu_tx_byte_inner(CalypsoSim *s, uint8_t b)
{
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    if (s->apdu_pos > 0 && now - s->last_tx_ns > APDU_ABANDON_NS) {
        SIM_LOG("APDU abandonne par le terminal apres %d octet(s) (silence %lld ms) : resynchronisation",
                s->apdu_pos, (long long)((now - s->last_tx_ns) / 1000000));
        s->apdu_pos = 0;
        s->apdu_expected = 0;
    }
    s->last_tx_ns = now;
    if (s->apdu_pos < APDU_MAX_LEN) {
        s->apdu[s->apdu_pos++] = b;
    }
    if (s->apdu_pos == 5) {

        uint8_t ins = s->apdu[1];
        bool is_outgoing_data =
            (ins == 0xA4) || (ins == 0xD6) || (ins == 0xDC) ||
            (ins == 0x20) || (ins == 0x24) || (ins == 0x88);
        if (is_outgoing_data && s->apdu[4] != 0) {
            s->apdu_expected = 5 + s->apdu[4];
        } else {
            s->apdu_expected = 5;
        }

        if (is_outgoing_data && s->apdu[4] != 0) {
            rx_push(s, ins);
            raise_rx_irq(s);
        }
    }
    if (s->apdu_pos == s->apdu_expected) {
        dispatch_apdu(s);
        s->apdu_pos = 0;
        s->apdu_expected = 0;
    }
}

uint16_t calypso_sim_reg_read(CalypsoSim *s, hwaddr off)
{
    switch (off) {
    case CALYPSO_SIM_REG_CMD:
        return s->cmd;
    case CALYPSO_SIM_REG_STAT: {
        uint16_t v = 0;
        if (s->powered) v |= CALYPSO_SIM_STAT_NOCARD;
        v |= CALYPSO_SIM_STAT_TXPAR;
        if (rx_count(s) == 0) v |= CALYPSO_SIM_STAT_FIFOEMPTY;
        if (rx_count(s) >= RX_FIFO_SIZE - 1) v |= CALYPSO_SIM_STAT_FIFOFULL;
        return v;
    }
    case CALYPSO_SIM_REG_CONF1:  return s->conf1;
    case CALYPSO_SIM_REG_CONF2:  return s->conf2;
    case CALYPSO_SIM_REG_IT: {
        refresh_it_rx(s);
        uint16_t v = s->it;

        uint16_t edge_seen = v & (CALYPSO_SIM_IT_NATR |
                                  CALYPSO_SIM_IT_WT   |
                                  CALYPSO_SIM_IT_OV   |
                                  CALYPSO_SIM_IT_TX);

        s->it &= ~edge_seen;
        update_irq(s);

        static unsigned itrd;
        if (itrd++ < 40) {
            uintptr_t io_pc = current_cpu ? current_cpu->mem_io_pc : 0;
            fprintf(stderr,
                    "[sim] SIM_IT read=0x%04x rx_count=%d edge_cleared=0x%04x "
                    "post_it=0x%04x mem_io_pc=0x%lx\n",
                    v, rx_count(s), edge_seen, s->it,
                    (unsigned long)io_pc);
        }
        return v;
    }
    case CALYPSO_SIM_REG_DRX: {
        uint8_t b = 0;
        rx_pop(s, &b);
        update_irq(s);

        return b | (1 << 8);
    }
    case CALYPSO_SIM_REG_DTX:    return 0;
    case CALYPSO_SIM_REG_MASKIT: return s->maskit;
    case CALYPSO_SIM_REG_IT_CD:  return s->it_cd;
    default: return 0;
    }
}

void calypso_sim_reg_write(CalypsoSim *s, hwaddr off, uint16_t val)
{
    switch (off) {
    case CALYPSO_SIM_REG_CMD:
        s->cmd = val;
        if (val & CALYPSO_SIM_CMD_START) {
            s->powered = true;
            SIM_LOG("CMDSTART → ATR delivered (synchronous)");

            deliver_atr(s);
        }
        if (val & CALYPSO_SIM_CMD_STOP) {
            s->powered = false;
            SIM_LOG("CMDSTOP");
        }
        if (val & (CALYPSO_SIM_CMD_CARDRST | CALYPSO_SIM_CMD_IFRST)) {
            /* [2026-10-21] Le driver SIM de TI (Magnetite, sim.c SIM_ManualStart)
             * n'ecrit jamais START : CLKEN, STOP, puis SWRST en mode automatique
             * et il attend l'ATR. Un reset avec l'horloge du module active
             * alimente donc la carte et livre l'ATR (osmocom ecrit START avant,
             * rien ne change pour lui). */
            /* [2026-10-21 11:30] Livrer l'ATR ici (SWRST + CLKEN) cassait le
             * driver TI : la carte repondait avant que SIM_ManualStart ait mis
             * VCC, SIO et l'horloge, et la sequence ne finissait jamais. L'ATR
             * du mode manuel part sur CONF1 (SCLKEN / SRSTLEV, ci-dessous). */
            SIM_LOG("RESET → ATR delivered (synchronous) pc=0x%08x lr=0x%08x", sim_guest_pc(), sim_guest_lr());
            s->apdu_pos = 0;
            s->apdu_expected = 0;
            s->resp_len = 0;
            s->rx_head = s->rx_tail = 0;
            s->selected_df = 0x3F00;
            s->selected_ef = 0x3F00;

            if (s->powered) deliver_atr(s);
        }
        break;
    case CALYPSO_SIM_REG_STAT:    s->stat   = val; break;
    case CALYPSO_SIM_REG_CONF1: {
        /* [2026-10-21] Mode manuel (BYPASS) du driver SIM de TI : il alimente
         * la carte (SVCCLEV), lache SIO, lance l'horloge (SCLKEN) et attend
         * l'ATR, puis leve SRSTLEV s'il ne vient pas. Sur silicium la carte
         * repond a la sortie de reset : on livre l'ATR au front montant de
         * SCLKEN ou de SRSTLEV, une fois par reset. */
        uint16_t old = s->conf1;
        s->conf1 = val;
        if (val != old) {
            SIM_LOG("CONF1 <- 0x%04x (powered=%d rx=%d)", val, s->powered, rx_count(s));
        }
        if (val & CALYPSO_SIM_CONF1_SVCCLEV) {
            s->powered = true;
        }
        /* warm reset (SIM_WARMReset : SRSTLEV bas 400 ETU puis haut) : nouvel ATR */
        if (!(val & CALYPSO_SIM_CONF1_SRSTLEV) && (old & CALYPSO_SIM_CONF1_SRSTLEV)) {
            s->atr_manual_done = false;
            s->rx_head = s->rx_tail = 0;
            s->apdu_pos = 0;
            s->apdu_expected = 0;
        }
        if ((val & CALYPSO_SIM_CONF1_BYPASS) && s->powered && !s->atr_manual_done &&
            (((val & CALYPSO_SIM_CONF1_SCLKEN) && !(old & CALYPSO_SIM_CONF1_SCLKEN)) ||
             ((val & CALYPSO_SIM_CONF1_SRSTLEV) && !(old & CALYPSO_SIM_CONF1_SRSTLEV)))) {
            s->atr_manual_done = true;
            SIM_LOG("mode manuel (TI) : ATR livre sur CONF1=0x%04x", val);
            schedule_atr(s);
        }
        if (!(val & CALYPSO_SIM_CONF1_SVCCLEV) && (old & CALYPSO_SIM_CONF1_SVCCLEV)) {
            s->atr_manual_done = false;
        }
        break;
    }
    case CALYPSO_SIM_REG_CONF2:   s->conf2  = val; break;
    case CALYPSO_SIM_REG_IT:           break;
    case CALYPSO_SIM_REG_DRX:            break;
    case CALYPSO_SIM_REG_DTX:
        SIM_LOG("DTX <- 0x%02x", (unsigned)(val & 0xff));
        apdu_tx_byte(s, (uint8_t)(val & 0xFF));
        /* [2026-10-21] Octet parti : IT TX (le driver TI, sim32.c, n'envoie
         * l'octet suivant d'une commande que sur cette IT, STAT TXPAR a 1).
         * Acquittee a la lecture de SIM_IT comme NATR/WT/OV. */
        /* [2026-10-24] ... mais au bout d'un temps caractere, pas tout de suite. Levee aussitot, l'IT faisait partir
         * les 5 octets d'en-tete en quelques us et l'ACK suivait 1,5 ms apres le PREMIER octet. Or SIM_Command_Base
         * ecrit le premier octet (SIM_WriteBuffer) puis seulement passe moderx a 6 (attente de l'ACK) : une tache SIM
         * preemptee 1,5 ms entre les deux recevait l'ACK en moderx 5 (range comme SW1), attendait un ACK qui ne
         * venait plus, SIM_ERR_WAIT, en-tete renvoye -> desynchronisation (voir APDU_ABANDON_NS). A 9600 bauds un
         * octet dure ~1 ms (CALYPSO_SIM_US_OCTET), l'en-tete ~5 ms comme sur une vraie carte. */
        timer_mod_ns(s->tx_timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + (int64_t)us_octet() * 1000);
        break;
    case CALYPSO_SIM_REG_MASKIT: {

        s->maskit = val;
        update_irq(s);
        bool wt_unmasked = (val & CALYPSO_SIM_IT_WT) == 0;
        bool wt_not_pending = (s->it & CALYPSO_SIM_IT_WT) == 0;
        if (wt_unmasked && wt_not_pending && rx_count(s) == 0) {
            schedule_wt(s);
        }
        break;
    }
    case CALYPSO_SIM_REG_IT_CD:   s->it_cd  = val; break;
    default: break;
    }
}

static void load_config_from_file(CalypsoSim *s, const char *path)
{
    FILE *fp = fopen(path, "r");
    if (!fp) {
        SIM_LOG("config %s not found — keeping default file system", path);
        return;
    }
    char line[256];
    bool in_test_sim = false;
    while (fgets(line, sizeof(line), fp)) {
        char *p = line;
        while (*p == ' ' || *p == '\t') p++;
        if (strncmp(p, "test-sim", 8) == 0) {
            in_test_sim = true;
            continue;
        }
        if (strncmp(p, "sms-service-center ", 19) == 0) {   /* [2026-10-22] -> EF_SMSP, voir rec_store */
            sscanf(p + 19, "%23s", g_sim_smsc);
            SIM_LOG("centre SMS (EF_SMSP) depuis %s : %s", path, g_sim_smsc);
            continue;
        }
        if (!in_test_sim) continue;

        if (line[0] != ' ' && line[0] != '\t' && line[0] != '\n') {
            in_test_sim = false;
            continue;
        }
        if (strncmp(p, "imsi ", 5) == 0) {
            const char *imsi = p + 5;
            uint8_t bcd[9];
            if (encode_imsi_bcd(imsi, bcd) > 0) {
                SimFile *ef = find_file(0x6F07, 0x7F20);
                if (ef) {
                    memcpy(ef->data, bcd, 9);
                    ef->size = 9;
                    SIM_LOG("EF_IMSI loaded from %s: %.15s", path, imsi);
                }
            }
        } else if (strncmp(p, "ki comp128 ", 11) == 0) {
            const char *hex = p + 11;
            int n = 0;
            while (n < 16 && *hex) {
                while (*hex == ' ') hex++;
                if (!*hex) break;
                unsigned v;
                if (sscanf(hex, "%2x", &v) != 1) break;
                s->ki[n++] = (uint8_t)v;
                hex += 2;
            }
            if (n == 16) {
                s->ki_valid = true;
                SIM_LOG("Ki loaded from %s (16 bytes)", path);
            }
        }
    }
    fclose(fp);
}

CalypsoSim *calypso_sim_new(qemu_irq sim_irq)
{
    CalypsoSim *s = g_new0(CalypsoSim, 1);
    s->irq = sim_irq;
    s->atr_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, deliver_atr, s);
    s->resp_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, resp_fire, s);
    s->wt_timer  = timer_new_ns(QEMU_CLOCK_VIRTUAL, fire_wt,     s);
    s->tx_timer  = timer_new_ns(QEMU_CLOCK_VIRTUAL, tx_fire,     s);

    s->selected_df = 0x3F00;
    s->selected_ef = 0x3F00;

    const char *cfg_path = getenv("CALYPSO_SIM_CFG");
    if (cfg_path && *cfg_path) {
        load_config_from_file(s, cfg_path);
    } else {
        SIM_LOG("CALYPSO_SIM_CFG unset — keeping default IMSI/Ki");
    }
    return s;
}
