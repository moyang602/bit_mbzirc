cmake_minimum_required(VERSION 2.8)
project(XmlBaseLIB)
include_directories(./)

aux_source_directory(./ LIB_SRCS)

add_library(XmlBase STATIC ${LIB_SRCS})
