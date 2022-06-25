---

toc: true
layout: post
description: Compile Latex file using Github Action
categories: [Latex][Github Action]
image: 
title: Compile Latex file using Github Action

---
# Compiling Latex documents using Github Action

GitHub Actions provides full access to the runner at your disposal, and one thing you may want to do is 
make commits in a workflow run and push it back up to GitHub automatically. I'm going to show a simple 
example where we run the date unix command, save the contents to a file, and push it back to the master branch.
Example workflow

## Using Makefile
* Use make file to automate all the actions which needs to be done after checking out the repo.
* Here is a Makefile for compiling latex file
```Makefile
filename=main

pdf:
	mkdir -p build
	pdflatex --output-directory build ${filename}
	bibtex build/${filename}||true
	pdflatex --output-directory build ${filename}
	pdflatex --output-directory build ${filename}
	mv build/${filename}.pdf .

read:
	evince build/${filename}.pdf &

clean:
	rm -f build/${filename}.{ps,pdf,log,aux,out,dvi,bbl,blg}


```
## Installing latex commands in Github Action
Inorder to use **pdflatex** in the ubuntu machine generted for compilation first you need to install the necessary packages 
in the ubuntu machine .


## Using git commit in GitHub Actions

The following is a workflow which on push will do the following:

    checkout the repo
    install latex dependency
    run make command and compile
    setup git config
    commit the changed file and push it back to master

```yml
name: Makefile CI

on:
  push:
    branches: [ "master" ]
  pull_request:
    branches: [ "master" ]

jobs:
  build:

    runs-on: ubuntu-latest

    steps:
    - uses: actions/checkout@v3
    
    - name: Install dependencies
      run: sudo apt-get install texlive-latex-base texlive-fonts-recommended texlive-fonts-extra texlive-latex-extra
  
    - name: Compile
      run: make
      
    - name: setup git config
      run: |
        # setup the username and email. I tend to use 'GitHub Actions Bot' with no email by default
        git config user.name "GitHub Actions Bot"
        git config user.email "<>"

    - name: commit
      run: |
        # Stage the file, commit and push
        git add main.pdf
        git commit -m "new main pdf commit"
        git push origin master


```

[1] https://lannonbr.com/blog/2019-12-09-git-commit-in-actions/
[2] [Example github page](https://github.com/sthoduka/designing_datasets/tree/master/.github/workflows)
