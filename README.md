Skatemate Software
==================

## Clone Instructions
```
git clone https://github.com/noah95/skatemate-sw
cd skatemate-sw
git submodule update --init --recursive
```

## Apply ChibiOS Patch
```
cd motorcontrol/ChibiOS/
git apply ../../ChibiOS_cmsis_patch.patch
```

## Compile motorcontrol
```
cd motorcontrol
make
```