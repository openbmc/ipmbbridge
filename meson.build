project(
  'ipmbbridge',
  'cpp',
  version: '0.1',
  meson_version: '>=0.58.0',
  default_options: [
    'warning_level=3',
    'werror=true',
    'cpp_std=c++20',
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

if cpp.has_header('nlohmann/json.hpp')
    nlohmann_json_dep = declare_dependency()
else
    subproject('nlohmann-json')
    nlohmann_json_dep = declare_dependency(
        include_directories: include_directories(
            'subprojects/nlohmann-json/single_include',
            'subprojects/nlohmann-json/single_include/nlohmann',
        )
    )
endif

boost_dep = dependency(
  'boost',
  modules: ['coroutine'],
  include_type: 'system',
)
i2c_dep = cpp.find_library('i2c')
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
