# SPDX-License-Identifier: CC0-1.0

ipa_name = 'ipa_ipu7'

ipu7_ipa_sources = files([
    'ipu7.cpp',
    'ipu7_isp.cpp',
])

mod = shared_module(ipa_name, ipu7_ipa_sources,
                    name_prefix : '',
                    include_directories : [ipa_includes],
                    dependencies : [libcamera_private, libipa_dep],
                    install : true,
                    install_dir : ipa_install_dir)

if ipa_sign_module
    custom_target(ipa_name + '.so.sign',
                  input : mod,
                  output : ipa_name + '.so.sign',
                  command : [ipa_sign, ipa_priv_key, '@INPUT@', '@OUTPUT@'],
                  install : false,
                  build_by_default : true)
endif

subdir('data')

ipa_names += ipa_name
