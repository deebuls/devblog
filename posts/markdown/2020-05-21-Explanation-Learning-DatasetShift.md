---
aliases:
- /dataset/2020/05/21/Explanation-Learning-DatasetShift
categories:
- dataset
date: '2020-05-21'
description: A explanation of dataset shifts in learning problems
image: images/domain_shift.png
layout: post
title: Dataset shift
toc: true

---

# Dataset shift

Dataset shift is still an unsolved problems when it comes to deploying learning models "in the wild".

There are 2 different categories of dataset shift
* Co-variate shift
* label shift

Let $ \mathbb{S} $ be the source data distribution and $ \mathbb{T} $ be the target data distribution.
If we denote the input variables as x and output variables as y, then 

## Covariate shift
$$ s(x) \neq t(x) $$
input distribution of both source and target are different

but

$$ s(y|x) = t(y|x) $$
conditional output distirbution is invariant to dataset shift.

## Label shift
$$ s(y) \neq t(y) $$
output distribution of both source and target are different

but 

$$ s(x|y) = t(x|y) $$
conditional input distirbution is invariant to dataset shift



|  | Covariate Shift | Label Shift |
|-|-|-|
| input distribution | $s(x) \neq t(x)$ | $?$ |
| output distribution | $?$ | $s(y) \neq t(y)$ |
| conditional output distribution | $s(y\vert x) = t(y \vert x)$ | $?$ | 
| conditional input Distribution | $?$ | ${s(x \vert y) = t(x \vert y)}$ |

## Examples 
ToDo

## Simluated Dataset {ReDo with examples}
The problem can be simulated in image based calssification dataset like MNIST and CIFAR.

### Tweak-One shift
refers to the case where we set a class to have probability $ p > 0.1$, while the distribution
over the rest of the classes is uniform. 
### Minority-Class Shiftis 
A more general version of Tweak-One shift, where a fixed number of classes to have probability
$p < 0.1$, while the distribution over the rest of the classes isuniform. 
### Dirichlet shift
we draw a probability vector $p$ from the Dirichlet distribution with concentration parameter
set to $\alpha$ for all classes, before including sample points which correspond to the multinomial
label variable according top. 

