Skatemate Software
==================

## Motorcontrol Build Instructions
```
git clone https://github.com/noah95/skatemate-sw/
cd skatemate-sw/
git submodule update --init --recursive
cd motorcontrol/ChibiOS/
git apply  --ignore-space-change --ignore-whitespace ../../ChibiOS_cmsis_patch.patch --reject
cd ..
make
```