---
aliases:
- /uncertainty/metrics/2021/08/08/Neurips-Competition-Uncertainty
categories:
- uncertainty
- metrics
date: '2021-08-08'
description: Neurips Competition Summary
layout: post
title: Neurips Competition Uncertainty Estimation
toc: true

---

# Notes on Neurips Competition


I am participating in the [Neurips Bayesian Deep Learning Competition](https://izmailovpavel.github.io/neurips_bdl_competition/), I will like to journal my notes here 

## Idea

Broad idea is to use Evidential loss function, Dropout, TTA combination.


## Journal 
- 8th August
    - Working in superconvergence 
- 13th August
    - Roll back to pytorch-cifar and modifications
- 14th August
    - Training on evidential loss reaches only 83% accuracy in 300 epochs. 
    - Why is evidental loss reducing the accuracy 
- 15th August
    - Training with CE, AdamW, OnecylceLR? Can we improve training speed. 
- 17th August
    - Dirichlet loss function. 
- 18th August
    - Dirichlet + Mixup : best results, touched 90%
    
## Reference

- [Super Convergence](https://gist.github.com/aakashns/90c13a903ff510c5baa72293fea72952)
    - cifar10, pytorchdata, cifar training, wideresenet, 92 accuracy
- [Pytorch Cifar SOA]()
    - pytorch cifar10, all models, 
- [Mixup ](https://github.com/facebookresearch/mixup-cifar10/blob/master/train.py)
    - pytorch mixup data combining while training


## Pytorch cifar 

| Model | data | criterion | optim | scheduler | epochs | accuracy | link | Notes |
|-----|------|------|-----|-----|-----|-----|-----|-----|
| Resnet18 | pytorch | cross-entropy | SGD | annealing-200 | 200 | 94 | [1](https://www.comet.ml/deebuls/cifar10-neurips/6737083a2e144c2c94ee2b191fb43b77?experiment-tab=stdout) | |
| Resnet20 | pytorch | cross-entropy | SGD | annealing-200 | 200 | 89 | [1](https://www.comet.ml/deebuls/cifar10-neurips/4ea079dd7dad4a19ae20b3c6ee1ee12c?experiment-tab=stdout) | |
| Resnet20 | tf      | cross-entropy | SGD | annealing-200 | 200 | 90 | [1](https://www.comet.ml/deebuls/cifar10-neurips/37242e3a6a504dd786c8f09baeece0f6?experiment-tab=stdout) | |
| Resnet20 | tf      | Evidential    | SGD | annealing-200 | 600 | 73/??/83 | [1](https://www.comet.ml/deebuls/cifar10-neurips/7625666bc7a44c62b53daa196c3b0b13?experiment-tab=stdout)[]()[]() | Added randmErasing |
| Resnet20 | tf      | Label smooting| SGD | annealing-200 | 200 |  | ?? | ?? |
| Resent20 | tf      | cross-entropy | AdamW | 1 cycle     | 30  |83 | [1](https://www.comet.ml/deebuls/cifar10-neurips/beffe711cde444c0910da49c682d0398?experiment-tab=stdout) | |
| Resnet20 | tf      | cross-entropy | AdamW | 1 cycle     | 100 |88 | [1](https://www.comet.ml/deebuls/cifar10-neurips/3e42b4ce283c4d069657418ef64a4a79)|max_lr = 0.01| 
| Resnet20 | tf      | cross-entropy | AdamW | 1 cycle     | 30  |50 | [1](https://www.comet.ml/deebuls/cifar10-neurips/17e7856afe54447bb245cfcf43056742) | max_lr=0.1 | 
| Resnet20 | tf      | cross-entropy | AdamW | 1 cycle     | 30  |80 | [1]() | max_lr=0.05 | 
| Resnet20 | tf      | Evidential    | AdamW | 1 cycle     | 30  |69 | [1]() | max_lr=0.05 | 
| Resnet20 | tf      | Evidential    | AdamW | annealing-200 | 200  |75 | [1](https://www.comet.ml/deebuls/cifar10-neurips/5d1fe7b3b4f1400b840b41a227c5a092) | max_lr=0.01 | 
| Resnet20 | tf      | cross-entropy | AdamW | 1 cycle     | 200 |89 | [1](https://www.comet.ml/deebuls/cifar10-neurips/d48a0594b3064f2680d4147ae08abaa5)|max_lr = 0.05| 
| Resnet20 | tf      | cross-entropy | AdamW | 1 cycle     | 200 |89 | [1]()|max_lr = 0.05, randomErase| 




