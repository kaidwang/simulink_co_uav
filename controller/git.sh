#!/bin/bash
echo "gaoling"
git add ./
git commit -m "update param"
git remote set-url origin git@github.com:BIT-aerial-robotics/controller_swarmUAV.git
git push origin main
