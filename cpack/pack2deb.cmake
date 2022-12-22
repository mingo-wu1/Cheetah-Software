##########################################
##生成deb包##
##########################################

set(PROJECT_NAME bzl-cheetah)

#准备的版本设置
EXECUTE_PROCESS(COMMAND git describe --abbrev=0
        TIMEOUT 5
        OUTPUT_VARIABLE GIT_VERSION
        OUTPUT_STRIP_TRAILING_WHITESPACE
        )

#说明要生成的是deb包
set(CPACK_GENERATOR "DEB")

############下面是设置debian/control文件中的内容
#设置版本信息
#set(CPACK_PACKAGE_VERSION_MAJOR "${_VERSION_MAJOR}")
#set(CPACK_PACKAGE_VERSION_MINOR "${_VERSION_MINOR}")
#set(CPACK_PACKAGE_VERSION_PATCH "${_VERSION_PATCH}")

#设置架构
if(${CMAKE_SYSTEM_PROCESSOR} STREQUAL aarch64)
    set(CPACK_DEBIAN_PACKAGE_ARCHITECTURE arm64)
else()
    set(CPACK_DEBIAN_PACKAGE_ARCHITECTURE amd64)
endif()

#设置安装包的包名，打好的包将会是packagename-version-archtype.deb，如果不设置，默认是工程名
set(CPACK_PACKAGE_FILE_NAME
        ${PROJECT_NAME}-${GIT_VERSION}-${CPACK_DEBIAN_PACKAGE_ARCHITECTURE})

#设置程序名，就是程序安装后的名字
set(CPACK_DEBIAN_PACKAGE_NAME bzl-cheetah)

#设置依赖
set(CPACK_DEBIAN_PACKAGE_DEPENDS "")

#设置section
set(CPACK_DEBIAN_PACKAGE_SECTION "devel")

#设置priority
set(CPACK_DEBIAN_PACKAGE_PRIORITY "optional")

#设置description
set(CPACK_PACKAGE_DESCRIPTION "bzl-cheetah")

#设置联系方式
set(CPACK_PACKAGE_CONTACT "wuchunming02@countrygarden.com.cn")

#设置维护人
set(CPACK_DEBIAN_PACKAGE_MAINTAINER "")

set(CPACK_DEBIAN_PACKAGE_HOMEPAGE "")

set(CPACK_SET_DESTDIR ON)

include(CPack)
