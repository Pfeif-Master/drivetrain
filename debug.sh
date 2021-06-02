#!/bin/bash
gcc -g -DDEBUG -o priv_test_drivetrain main_drivetrain.c drivetrain.c drivetrain.h
./priv_test_drivetrain
