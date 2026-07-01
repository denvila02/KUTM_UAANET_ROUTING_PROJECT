build_lib_example(
  NAME aodvetx
  SOURCE_FILES aodvetx.cc
  LIBRARIES_TO_LINK
    ${libwifi}
    ${libinternet}
    ${libaodvetx}
    ${libinternet-apps}
)
