build_lib_example(
  NAME qspu
  SOURCE_FILES qspu.cc
  LIBRARIES_TO_LINK
    ${libwifi}
    ${libinternet}
    ${libqspu}
    ${libinternet-apps}
)
