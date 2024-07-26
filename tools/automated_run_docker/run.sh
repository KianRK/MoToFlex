#!/bin/sh

# compile the MoToFlex simulator
if [ ! -d $MOTOFLEX_ROOT/tools/ode-0.16.4/ ]; then
    cd $MOTOFLEX_ROOT/tools && tar xvf ode-0.16.4.tar
    cd $MOTOFLEX_ROOT/tools/ode-0.16.4/ && ./bootstrap && ./configure --with-pic --disable-demos --disable-asserts && make -j8
fi

cd $MOTOFLEX_ROOT/Simulator && make

cd $MOTOFLEX_ROOT
python3 tools/train_motoflex.py
