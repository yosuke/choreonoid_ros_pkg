#!/bin/bash

run_args=()

# filter ros related arguments (e.g. __log:=)
for arg in $@; do
    if [[ $arg != "__"* ]]
    then
        run_args=("${run_args[@]}" $arg)
    fi
done

choreonoid "${run_args[@]}"
