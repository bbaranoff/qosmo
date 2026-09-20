# Couche 1 « C54x » : le DSP TMS320C54x emule, executant la vraie mask-ROM TI.
#
# [2026-09-16] Remontee depuis qosmo-dsp. libosmocoding est exigee par
# calypso_bsp.c (gsm0503_rach_ext_encode) : required:true pour que le configure
# echoue tout de suite plutot qu'a l'edition de liens.
osmocoding = dependency('libosmocoding', required: true)

arm_ss.add(when: 'CONFIG_CALYPSO', if_true: [
  files(
    'calypso_c54x.c',
    'c54x_exec.c',
    'c54x_decode.c',
    'c54x_mem.c',
    'c54x_irq.c',
    'c54x_probes.c',
    'calypso_arm2dsp.c',
    'calypso_mailbox.c',
    'calypso_fbsb.c',
    'calypso_dma.c',
    'calypso_bsp.c',
    'calypso_rif.c',
    'calypso_gmsk.c',
    'calypso_rhea_dma.c',
    'calypso_tint0.c',
    'calypso_twl3025.c',
    'calypso_full_pcb.c',
    'calypso_tint0_glue.c',
    'calypso_l1_dsp_register.c',
  ),
  osmocoding,
])
