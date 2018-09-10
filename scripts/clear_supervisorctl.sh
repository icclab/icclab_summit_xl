#!/bin/bash
echo $(sudo supervisorctl status | grep "STOPPED" | awk '{gsub("STOPPED", "");print'} | awk '{gsub("Not started", "");print}') | grep -o -E "[a-f0-9]{8}-([a-f0-9]{4}-){3}[a-f0-9]{12}:" | awk '{gsub(":", "");print}' | xargs sudo supervisorctl remove
