#   BSD LICENSE
#
#   Copyright(c) 2017- Intel Corporation. All rights reserved.
#   All rights reserved.
#
#   Redistribution and use in source and binary forms, with or without
#   modification, are permitted provided that the following conditions
#   are met:
#
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in
#       the documentation and/or other materials provided with the
#       distribution.
#     * Neither the name of Intel Corporation nor the names of its
#       contributors may be used to endorse or promote products derived
#       from this software without specific prior written permission.
#
#   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
#   A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
#   OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
#   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
#   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
#   DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
#   THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
#   (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
#   OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

pwd=$(pwd)

# checking if required environment variables are defined
if [ -z "${RTE_SDK}" ] || [ -z "${RTE_TARGET}" ] || [ -z "${RTE_SDKEX}" ]; then
	echo "At least one of the required environment variables (RTE_SDK, RTE_TARGET and RTE_SDKEX) are not defined!!!"
	exit
fi

# clean up build directory
if [ -d $RTE_SDKEX/build ];
then
	rm -f $RTE_SDKEX/build/lib/*
	rm -f $RTE_SDKEX/build/include/*
else
	mkdir -p $RTE_SDKEX/build
	mkdir -p $RTE_SDKEX/build/lib
	mkdir -p $RTE_SDKEX/build/include
fi

# gcc version detection
major=$(expr `gcc -dumpversion | cut -f1 -d.` - "0")
minor=$(expr `gcc -dumpversion | cut -f2 -d.` - "0")
stack_protector=-fstack-protector
if [ $major -gt 4 ]; then
   stack_protector=-fstack-protector-strong
else
   if [ $major -eq 4 ] && [ $minor -ge 9 ]; then
		stack_protector=-fstack-protector-strong
   fi
fi
export stack_protector=$stack_protector

# copy new ethdev header files into build/include
cp $RTE_SDKEX/lib/librte_ether_ex/*.h $RTE_SDKEX/build/include
cp $RTE_SDKEX/lib/librte_vni_util/*.h $RTE_SDKEX/build/include

# build ethdev_ex library
cd $RTE_SDKEX/lib/librte_ether_ex
echo "============build librte_ethdev_ex==================="
make
mv *.a $RTE_SDKEX/build/lib/.
mv *.o $RTE_SDKEX/build/lib/.

# build vni_util library
cd $RTE_SDKEX/lib/librte_vni_util
echo "============build librte_vni_util==================="
make
mv *.a $RTE_SDKEX/build/lib/.
mv *.o $RTE_SDKEX/build/lib/.

# build ixgbe_pmd_ex/ixgbevf_pmd_ex libraries
cd $RTE_SDKEX/drivers/net/ixgbe_ex
echo "============build librte_pmd_ixgbe_ex==================="
make
mv *.o $RTE_SDKEX/build/lib/.
mv *.a $RTE_SDKEX/build/lib/.

# build i40e_pmd_ex/i40evf_pmd_ex libraries
cd $RTE_SDKEX/drivers/net/i40e_ex
echo "============build librte_pmd_i40e_ex==================="
make
mv *.o $RTE_SDKEX/build/lib/.
mv *.a $RTE_SDKEX/build/lib/.

cd $pwd

# Build DPDM integrated testpmd

# remove the old test-pmd if it exists
# 
[ -d $RTE_SDKEX/app/test-pmd ] && rm -r $RTE_SDKEX/app


# copy 
mkdir $RTE_SDKEX/app
cp -r $RTE_SDK/app/test-pmd $RTE_SDKEX/app/.

# apply patch
dpdk=$(echo $RTE_SDK | grep -Po '(dpdk-\d+.\d+(-rc\d*)*)')
cd $RTE_SDKEX
patch -p1 < $RTE_SDKEX/doc/testpmd-${dpdk}.patch

# build integrated test-pmd
cd $RTE_SDKEX/app/test-pmd
make
