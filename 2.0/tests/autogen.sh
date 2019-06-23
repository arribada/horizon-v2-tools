#!/bin/sh
# you can either set the environment variables AUTOCONF, AUTOHEADER, AUTOMAKE,
# ACLOCAL, AUTOPOINT and/or LIBTOOLIZE to the right versions, or leave them
# unset and get the defaults

autoreconf --verbose --force --install --make || {
 echo 'autogen.sh failed';
 exit 1;
}

echo Run:
echo ./configure --host=$ARCH_PREFIX --with-libtool-sysroot=$PKG_CONFIG_SYSROOT_DIR

echo
echo "Then type 'make' to compile this module."
echo
