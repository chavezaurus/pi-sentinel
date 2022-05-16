project('sentinel', 'c', 'cpp',
	default_options : [
		'werror=true',
		'warning_level=2',
		'cpp_std=c++17',
		'optimization=0',
	])

# pi-sentinel.cpp is the fully commented application
src_files = files([
	'pi-sentinel.cpp',
])

# Point your PKG_CONFIG_PATH environment variable to the
# libcamera install path libcamera.pc file ($prefix/lib/pkgconfig/libcamera.pc)
deps = [
      dependency('libcamera', required : true),
      dependency('threads'),
	  dependency('libv4l2'),
	  dependency('libjpeg'),
]

cpp_arguments = [ '-Wno-unused-parameter', '-Wno-psabi', ]

add_project_arguments(cpp_arguments, language : 'cpp')

# sentinel executable
sentinel = executable('sentinel', src_files,
                        dependencies : deps)
