---
toc: true
layout: post
description: How to use uncertainty estimates from deep neural networks
categories: [uncertainty]
image: 
title: Using uncertainties in DNN application
---

# Application of Uncertainty Estimation in DNN

## Objective 
We would like to look into two papers which have developed methodologies to use the uncertatiny estimated by the deep neural network(DNN) in their system.
The goal is to find the pros and cons of these methodologies

## Paper 1: Benchmarking uncertatinty estimation methods in Deep learning with Safety-Related Metrics
In this paper they propose two new Safety-Related Metrics: Remaining Error Rate(RER) and Remaining Accuracy Rate(Rate). Here is definition as per the paper:
> A system which relies on these estimates is expected to be in functional mode if the predictions are certain and in fall-back/mitigation mode 
if the prediction is uncertain. However, the most critical result regarding safety are the predictions where the model is
certain about its prediction but incorrect (CI). We call the ratio of the number of certain but
incorrect samples to all samples the Remaining Error Rate (RER). For minimizing the overall risk, it needs to be as low
as possible. Nonetheless, if a model would always give a low confidence as output, the system would constantly remain in
fall-back mode and will be unable to provide the intended functionality. Therefore, the ratio of the number of certain
and correct samples to all samples - we call it the Remaining Accuracy Rate (RAR) - needs to be as high as possible
to stay in performance mode for most of the time.

### Interpretation
The predictions from the DNN are classified into 4 sections as below (Table 1 from paper)
| | Certain | Uncertain |
|----|----|-----|
| Correct | **CC** | **UC** |
| Incorrect | **CI** | **UI** |

The definition of the metrics are :

$RER = \frac{CI}{CC+CI+UC+UI}$
$RAR = \frac{CC}{CC+CI+UC+UI}$

#### Pros
* Its a very simple metric. 
* Simplicity of metric is a very important thing for usability of metrics. 

#### Cons
* A minor issue will be on the threshold which seprates Certain vs Uncertain. Is it 99% or 90% etc. All will yield different results
* A major problem which we consider is the assumption in which the uncertatiny is being planned to be used in the system. 
> **A system which relies on these estimates is expected to be in functional mode if the predictions are certain and in fall-back/mitigation mode 
if the prediction is uncertain.** 
* This means that the system has 2 modes
    - A functional mode
    - A fall-back/mitigation mode
* Is this a safe assumption with regards to deployment of DNN?
*  Can an application deploying DNN have 2 modes ?
*  What should an autonomous car in fall-back mode do ?


:bangbang: | The assumption on how a system uses uncertatiny is that the system has 2 modes functional and fall-back
:---: | :---

## Paper 2 : Fail-Safe Execution of Deep learning based Systems through Uncertatiny Monitoring
* In this paper, as the title suggests they create a separate model called the *Supervisor Model* which will monitor the uncertainty of Deep learning and 
avoid any faults in the system
* What is a supervisor model :
> Network supervision can be viewed as a binary classification task: malicious samples, i.e., inputs which lead to a misclassification
>  (for classification problems) or to severe
imprecision (in regression problems) are positive samples that have to be rejected. Other samples, also called benign samples,
are negative samples in the binary classification task. An uncertainty based supervisor accepts an input i as a benign
sample if its uncertainty u(i) is lower than some threshold t. The choice of t is a crucial setting, as a high t will fail to
reject many malicious samples (false negatives) and a low t  will cause too many false alerts (false positives).

* Thus the supervisor is a binary classification task to avoid beningn samples. They also define a metric *S-Score* which combined measures the performance
of both the model and the supervisor model
* There is lot of similarity with respect to the above paper here also

#### Pros
* They have made a library out of it such that any model can be used.
* The threshold on which to make the decission is now being learned by the data.

#### Cons
* Again, these method is based on the assumption that the system which uses DNN has 2 modes of operation( normal mode and fall-back mode)


:bangbang: | The same assumption on how a system uses uncertatiny, that the system has 2 modes functional and fall-back
:---: | :---

## Conclusion

* All methods are based on the assumption that the system has 2 modes of operation
* The uncertatiny estimation is used to determine whethere the DNN output should be trusted or should be avoided

### This is not enough
* The methods which use DNN dont have a fall back mode. 
    * If there was an non DNN based method then by "First rule of Machine/Deep Learning" that will be used for solving the problem
* There can be argument to say that there are redundant DNN systems and this method can be used to kick-off redundant system
    * Even this argument is not valid as if you have redundant system, you should use all of them and make a decision

### Solutions
* The one solution which I have been workin is about not binarizing the probability but the propagating it through the system
* The best example is of the filters which have been developed over years to handle uncertain sensors.




## References
[1]M. Weiss and P. Tonella, “Fail-Safe Execution of Deep Learning based Systems through Uncertainty Monitoring,” arXiv:2102.00902 [cs], Feb. 2021, 
Accessed: Apr. 13, 2021. [Online]. Available: http://arxiv.org/abs/2102.00902.
[2]M. Henne, A. Schwaiger, K. Roscher, and G. Weiss, “Benchmarking Uncertainty Estimation Methods for Deep Learning With Safety-Related Metrics,” p. 8, 2020.

