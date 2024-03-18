export PATH=~/linux-develop/3568/buildroot/output/rockchip_rk3568/host/bin:$PATH

aarch64-buildroot-linux-gnu-gcc ec_master_test_RK3568_MADHT1505BA1.c  -I ../ethercat_igh/include/ -L ../ethercat_igh/lib/.libs -g -lethercat -o example

aarch64-buildroot-linux-gnu-gcc Rockchip_MADHT1505BA1.c -I ../ethercat_igh/include/ -L ../ethercat_igh/lib/.libs -g -lethercat -shared -fPIC -o libmadht1505ba1.so

aarch64-buildroot-linux-gnu-gcc rk_test_two.c  -I ../ethercat_igh/include/ -L ../ethercat_igh/lib/.libs -L ./ -g -lmadht1505ba1 -lethercat -o rk_test_two

aarch64-buildroot-linux-gnu-gcc rk_test.c  -I ../ethercat_igh/include/ -L ../ethercat_igh/lib/.libs -L ./ -g -lmadht1505ba1 -lethercat -o rk_test
