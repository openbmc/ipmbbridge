project(
  'ipmbbridge',
  'cpp',
  version: '0.1',
  meson_version: '>=1.1.1',
  default_options: [
    'warning_level=3',
    'werror=true',
    'cpp_std=c++23',
  ],
)

cpp = meson.get_compiler('cpp')
add_project_arguments(
  cpp.get_supported_arguments([
    '-DBOOST_ERROR_CODE_HEADER_ONLY',
    '-DBOOST_SYSTEM_NO_DEPRECATED',
    '-DBOOST_ALL_NO_LIB',
    '-DBOOST_NO_RTTI',
    '-DBOOST_NO_TYPEID',
    '-DBOOST_ASIO_DISABLE_THREADS',
  ]),
  language: 'cpp',
)

retry_delay_ms = get_option('retry-delay-ms')
if retry_delay_ms > 0
  add_project_arguments('-DRETRY_DELAY_MS=@0@'.format(retry_delay_ms), language: 'cpp')
endif


boost_dep = dependency(
  'boost',
  modules: ['coroutine'],
  include_type: 'system',
)
i2c_dep = cpp.find_library('i2c')
nlohmann_json_dep = dependency('nlohmann_json', include_type: 'system')
phosphor_logging_dep = dependency('phosphor-logging')
sdbusplus_dep = dependency('sdbusplus')
systemd_dep = dependency('systemd')

executable(
  'ipmbbridged',
  'ipmbbridged.cpp',
  'ipmbutils.cpp',
  dependencies: [
    boost_dep,
    i2c_dep,
    nlohmann_json_dep,
    phosphor_logging_dep,
    sdbusplus_dep,
  ],
  install: true,
)

install_data(
  'ipmb.service',
  install_dir: systemd_dep.get_variable('systemdsystemunitdir'),
)

install_data(
  'ipmb-channels.json',
  install_dir: get_option('datadir') / 'ipmbbridge',
)
