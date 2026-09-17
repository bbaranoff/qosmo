# Le lanceur `qosmo` : qemu-system-arm (machine calypso) + pty + osmocon.
#
# [2026-09-16] Repris de qosmo-grgsm/tools/qosmo-launch. La-bas il fallait le
# construire deux fois - une par fork, avec un ALIAS different - parce que le
# choix de la couche 1 etait un choix de depot. Ici il n'y en a qu'un, et
# QOSMO_DSP suit directement l'option de configure : le lanceur sait donc
# toujours quelle couche 1 le qemu qu'il lance embarque, sans qu'on le lui
# repete.
#
# `ninja install` le pose dans <prefix>/bin/qosmo, a cote de qemu-system-arm.
qosmo_version = 'local'
git_prog = find_program('git', required: false)
if git_prog.found()
  r = run_command(git_prog, '-C', meson.project_source_root(),
                  'rev-parse', '--short', 'HEAD', check: false)
  if r.returncode() == 0
    qosmo_version = r.stdout().strip()
  endif
endif

executable('qosmo', files('qosmo.c'),
  c_args: [
    '-DQOSMO_ALIAS="qosmo"',
    '-DQOSMO_TREE="' + meson.project_source_root() + '"',
    '-DQOSMO_DSP=' + (get_option('l1_dsp').enabled() ? '1' : '0'),
    '-DQOSMO_VERSION="' + qosmo_version + '"',
  ],
  install: true,
  install_dir: get_option('bindir'),
)
