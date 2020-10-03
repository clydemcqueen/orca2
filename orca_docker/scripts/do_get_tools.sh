#!/bin/bash -x

# Install locate
apt-get install -y locate
updatedb

# Install vim
apt-get install -y vim
