project(
    'ipmbbridge',
    'cpp',
    version: '0.1',
    meson_version: '>=1.1.1',
    default_options: [
        'warning_level=3',
        'werror=true',
        'cpp_std=c++23',
        'b_lto=true',
    ],
)

cpp = meson.get_compiler('cpp')
add_project_arguments(
    cpp.get_supported_arguments(
        [
            '-DBOOST_ERROR_CODE_HEADER_ONLY',
            '-DBOOST_SYSTEM_NO_DEPRECATED',
            '-DBOOST_ALL_NO_LIB',
            '-DBOOST_NO_RTTI',
            '-DBOOST_NO_TYPEID',
            '-DBOOST_ASIO_DISABLE_THREADS',
        ],
    ),
    language: 'cpp',
)


boost_dep = dependency(
    'boost',
    modules: ['coroutine', 'context'],
    include_type: 'system',
    required: false,
)

if not boost_dep.found()
    cmake = import('cmake')
    opt = cmake.subproject_options()
    opt.add_cmake_defines(
        {'BOOST_INCLUDE_LIBRARIES': 'asio;callable_traits;context;coroutine'},
    )
    boost_cmake = cmake.subproject('boost', required: true, options: opt)
    boost_asio = boost_cmake.dependency('boost_asio').as_system()
    boost_callable_traits = boost_cmake.dependency('boost_callable_traits').as_system()
    boost_context = boost_cmake.dependency('boost_context').as_system()
    boost_coroutine = boost_cmake.dependency('boost_coroutine').as_system()
    boost_dep = [
        boost_asio,
        boost_callable_traits,
        boost_context,
        boost_coroutine,
    ]
endif

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
