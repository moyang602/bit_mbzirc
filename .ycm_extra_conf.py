#!/usr/bin/env python

import os
import ycm_core

flags = [
'-Wall',
'-Wextra',
'-Werror',
'-fexceptions',
'-DNDEBUG',
'-std=c++11',
'-x',
'c++',
'-isystem',
'/usr/include',
'-isystem',
'/usr/local/include',
'-isystem',
'/opt/ros/' + os.getenv('ROS_DISTRO') + '/include',
'-isystem',
'/home/moyang/bit_mbzirc/devel/include',
'-isystem',
'/home/moyang/bit_mbzirc/src/asr_halcon_bridge/include',
'-isystem',
'/home/moyang/bit_mbzirc/src/bit_centernet/include',
'-isystem',
'/home/moyang/bit_mbzirc/src/bit_control/include',
'-isystem',
'/home/moyang/bit_mbzirc/src/bit_description/include',
'-isystem',
'/home/moyang/bit_mbzirc/src/bit_hardware_interface/include',
'-isystem',
'/home/moyang/bit_mbzirc/src/bit_motion/include',
'-isystem',
'/home/moyang/bit_mbzirc/src/bit_msgs/include',
'-isystem',
'/home/moyang/bit_mbzirc/src/bit_plan/include',
'-isystem',
'/home/moyang/bit_mbzirc/src/bit_task/include',
'-isystem',
'/home/moyang/bit_mbzirc/src/bit_vision/include',
'-isystem',
'/home/moyang/bit_mbzirc/src/dji_sdk/include',
'-isystem',
'/home/moyang/bit_mbzirc/src/laser_filters/include',
'-isystem',
'/home/moyang/bit_mbzirc/src/mer_test/include',
'-isystem',
'/home/moyang/bit_mbzirc/src/robot_pose_ekf/include',
'-isystem',
'/home/moyang/bit_mbzirc/src/room_coverage/include',
'-isystem',
'/home/moyang/bit_mbzirc/src/ur_modern_driver/include',
'-isystem',
'/home/moyang/bit_mbzirc/src/zed_wrapper/include'
]

compilation_database_folder = ''

if os.path.exists( compilation_database_folder ):
  database = ycm_core.CompilationDatabase( compilation_database_folder )
else:
  database = None

SOURCE_EXTENSIONS = [ '.cpp', '.cxx', '.cc', '.c' ]

def DirectoryOfThisScript():
  return os.path.dirname( os.path.abspath( __file__ ) )


def MakeRelativePathsInFlagsAbsolute( flags, working_directory ):
  if not working_directory:
    return list( flags )
  new_flags = []
  make_next_absolute = False
  path_flags = [ '-isystem', '-I', '-iquote', '--sysroot=' ]
  for flag in flags:
    new_flag = flag

    if make_next_absolute:
      make_next_absolute = False
      if not flag.startswith( '/' ):
        new_flag = os.path.join( working_directory, flag )

    for path_flag in path_flags:
      if flag == path_flag:
        make_next_absolute = True
        break

      if flag.startswith( path_flag ):
        path = flag[ len( path_flag ): ]
        new_flag = path_flag + os.path.join( working_directory, path )
        break

    if new_flag:
      new_flags.append( new_flag )
  return new_flags


def IsHeaderFile( filename ):
  extension = os.path.splitext( filename )[ 1 ]
  return extension in [ '.h', '.hxx', '.hpp', '.hh' ]


def GetCompilationInfoForFile( filename ):
  if IsHeaderFile( filename ):
    basename = os.path.splitext( filename )[ 0 ]
    for extension in SOURCE_EXTENSIONS:
      replacement_file = basename + extension
      if os.path.exists( replacement_file ):
        compilation_info = database.GetCompilationInfoForFile(
          replacement_file )
        if compilation_info.compiler_flags_:
          return compilation_info
    return None
  return database.GetCompilationInfoForFile( filename )


def FlagsForFile( filename, **kwargs ):
  if database:
    compilation_info = GetCompilationInfoForFile( filename )
    if not compilation_info:
      return None

    final_flags = MakeRelativePathsInFlagsAbsolute(
      compilation_info.compiler_flags_,
      compilation_info.compiler_working_dir_ )
  else:
    relative_to = DirectoryOfThisScript()
    final_flags = MakeRelativePathsInFlagsAbsolute( flags, relative_to )

  return {
    'flags': final_flags,
    'do_cache': True
  }
