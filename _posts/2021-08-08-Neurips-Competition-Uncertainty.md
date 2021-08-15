---

toc: true
layout: post
description: Neurips Competition Summary
categories: [uncertainty, metrics]
image: 
title: 

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
    
## Reference

- [Super Convergence](https://gist.github.com/aakashns/90c13a903ff510c5baa72293fea72952)
    - cifar10, pytorchdata, cifar training, wideresenet, 92 accuracy
- [Pytorch Cifar SOA]()
    - pytorch cifar10, all models, 


## Pytorch cifar 

| Model | data | criterion | optim | scheduler | epochs | accuracy | link | Notes |
|-----|------|------|-----|-----|-----|-----|-----|-----|
| Resnet18 | pytorch | cross-entropy | SGD | annealing-200 | 200 | 94 | [1](https://www.comet.ml/deebuls/cifar10-neurips/6737083a2e144c2c94ee2b191fb43b77?experiment-tab=stdout) | |
| Resnet20 | pytorch | cross-entropy | SGD | annealing-200 | 200 | 89 | [1](https://www.comet.ml/deebuls/cifar10-neurips/4ea079dd7dad4a19ae20b3c6ee1ee12c?experiment-tab=stdout) | |
| Resnet20 | tf      | cross-entropy | SGD | annealing-200 | 200 | 90 | [1](https://www.comet.ml/deebuls/cifar10-neurips/37242e3a6a504dd786c8f09baeece0f6?experiment-tab=stdout) | |
| Resnet20 | tf      | Evidential    | SGD | annealing-200 | 600 | 73/??/83 | [1](https://www.comet.ml/deebuls/cifar10-neurips/7625666bc7a44c62b53daa196c3b0b13?experiment-tab=stdout)[]()[]() | Added randmErasing |
| Resnet20 | tf      | Label smooting| SGD | annealing-200 | 200 |  | ?? | ?? |
| Resent20 | tf      | cross-entropy | AdamW | 1 cycle     | 30  |?? | [1](https://www.comet.ml/deebuls/cifar10-neurips/beffe711cde444c0910da49c682d0398?experiment-tab=stdout) | |

