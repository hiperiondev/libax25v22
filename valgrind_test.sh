#! /bin/bash

clear
valgrind --leak-check=full --show-leak-kinds=all --track-origins=yes --verbose Release/libax25v22 $1
rm vgcore*
