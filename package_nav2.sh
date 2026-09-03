#!/bin/bash
set -e

ROS_DISTRO=${ROS_DISTRO:-humble}
ARCH=$(dpkg --print-architecture 2>/dev/null || echo "arm64")

WS_DIR="${1:-$PWD}"
INSTALL_DIR="${WS_DIR}/install"
OUTPUT_DIR="${WS_DIR}/deb_packages"

echo "============================================================"
echo " Nav2 Debian Packager pro Jetson Orin NX"
echo " Filtr:        ros-${ROS_DISTRO}-nav*"
echo " Workspace:    ${WS_DIR}"
echo " Install dir:  ${INSTALL_DIR}"
echo " Výstupní dir: ${OUTPUT_DIR}"
echo " Distribuce:   ${ROS_DISTRO}"
echo " Architektura: ${ARCH}"
echo "============================================================"

if [ ! -d "${INSTALL_DIR}" ]; then
    echo "CHYBA: Složka '${INSTALL_DIR}' neexistuje!"
    echo "Nejprve spusťte 'colcon build' v kořenovém adresáři workspace."
    exit 1
fi

mkdir -p "${OUTPUT_DIR}"
SUCCESS_COUNT=0
SKIP_COUNT=0

for pkg_path in "${INSTALL_DIR}"/*; do
    [ -d "${pkg_path}" ] || continue
    PKG_NAME=$(basename "${pkg_path}")

    # Debian název balíčku
    DEB_PKG_NAME="ros-${ROS_DISTRO}-$(echo "${PKG_NAME}" | tr '_' '-')"

    # FILTR: Zpracovat pouze balíčky začínající na ros-<distro>-nav*
    case "${DEB_PKG_NAME}" in
        "ros-${ROS_DISTRO}-nav"*)
            ;;
        *)
            # Ostatní balíčky přeskočit
            continue
            ;;
    esac

    # Vyhledání package.xml
    PKG_XML="${pkg_path}/share/${PKG_NAME}/package.xml"
    if [ ! -f "${PKG_XML}" ]; then
        SRC_XML=$(find "${WS_DIR}/src" -maxdepth 3 -type f -name "package.xml" -path "*/${PKG_NAME}/package.xml" 2>/dev/null | head -n 1)
        if [ -n "${SRC_XML}" ] && [ -f "${SRC_XML}" ]; then
            PKG_XML="${SRC_XML}"
        fi
    fi

    if [ ! -f "${PKG_XML}" ]; then
        echo "--> [PŘESKOČENO] ${PKG_NAME}: Nenalezen package.xml."
        SKIP_COUNT=$((SKIP_COUNT + 1))
        continue
    fi

    RAW_VERSION=$(grep -oPm1 "(?<=<version>)[^<]+" "${PKG_XML}" 2>/dev/null || echo "1.0.0")
    VERSION="${RAW_VERSION}-custom"
    
    DESC_RAW=$(grep -oPm1 "(?<=<description>)[^<]+" "${PKG_XML}" 2>/dev/null | head -n 1 | tr '\n' ' ' | sed 's/[^a-zA-Z0-9 ._-]//g')
    DESCRIPTION="${DESC_RAW:-Custom Nav2 package with modified C++ headers for Jetson}"

    echo "--> Zpracovávám: ${DEB_PKG_NAME} (${VERSION})..."

    BUILD_ROOT="/tmp/deb_build_${PKG_NAME}"
    rm -rf "${BUILD_ROOT}"
    mkdir -p "${BUILD_ROOT}/DEBIAN"
    mkdir -p "${BUILD_ROOT}/opt/ros/${ROS_DISTRO}"

    # Zkopírování binárek, knihoven a upravených C++ headerů
    cp -rL "${pkg_path}"/* "${BUILD_ROOT}/opt/ros/${ROS_DISTRO}/"

    # Odstranění colcon interních skriptů
    rm -f "${BUILD_ROOT}/opt/ros/${ROS_DISTRO}"/setup.*
    rm -f "${BUILD_ROOT}/opt/ros/${ROS_DISTRO}"/local_setup.*
    rm -f "${BUILD_ROOT}/opt/ros/${ROS_DISTRO}"/_order_isolated.py
    rm -f "${BUILD_ROOT}/opt/ros/${ROS_DISTRO}"/COLCON_IGNORE
    rm -f "${BUILD_ROOT}/opt/ros/${ROS_DISTRO}"/.built_by

    # Debian metadata
    printf "Package: %s\nVersion: %s\nArchitecture: %s\nMaintainer: Jetson Developer <developer@orin.local>\nSection: misc\nPriority: optional\nProvides: %s\nReplaces: %s\nConflicts: %s\nDescription: %s\n" \
        "${DEB_PKG_NAME}" "${VERSION}" "${ARCH}" "${DEB_PKG_NAME}" "${DEB_PKG_NAME}" "${DEB_PKG_NAME}" "${DESCRIPTION}" > "${BUILD_ROOT}/DEBIAN/control"

    chmod 755 "${BUILD_ROOT}/DEBIAN"
    chmod 644 "${BUILD_ROOT}/DEBIAN/control"

    # Zabalení do .deb
    DEB_FILE="${OUTPUT_DIR}/${DEB_PKG_NAME}_${VERSION}_${ARCH}.deb"
    dpkg-deb --build --root-owner-group "${BUILD_ROOT}" "${DEB_FILE}" > /dev/null

    rm -rf "${BUILD_ROOT}"
    SUCCESS_COUNT=$((SUCCESS_COUNT + 1))
done

echo "============================================================"
echo " HOTOVO! Úspěšně zabaleno: ${SUCCESS_COUNT} balíčků."
echo " Umístění balíčků: ${OUTPUT_DIR}"
echo "============================================================"
