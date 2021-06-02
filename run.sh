#!/bin/bash
gcc -g -o priv_test_drivetrain main_drivetrain.c drivetrain.c drivetrain.h
./priv_test_drivetrain | less -r
