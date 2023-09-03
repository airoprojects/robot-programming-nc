#! /bin/bash

email='airoprojectsx@gmail.com'
username='airoprojects'
repository="robot-programming-nc"
token="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"

git remote set-url origin https://${token}@github.com/${username}/${repository}
git remote -v
