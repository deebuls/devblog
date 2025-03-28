---
aliases:
- /Reliability/2020/05/29/Explanation-DNN-Reliability
categories:
- Reliability
date: '2020-05-29'
description: Reliability of DNN
layout: post
title: Reliability in DNN systems(WIP)
toc: true

---

# What is Reliability?
Sub-system(DNN) and  System(using DNN)

According to [2] reliability is :
- Failure prevention
- Failure identificaiton
- Reliability monitoring


# Is it different from classical reliability?

# Software Reliability Assesment
- SRA is done for specific **operatioal profile**
- Profile is needed in testing to select or generate test cases in a way statistically similar to anticipated software use.
- observation of success/failure used to predict operational reliability
- MOST IMPORTANT - **operational profile** needs to be accurate otherwise reliability predictions cannot be trusted
- Generally, the likelihood of selecting an input from D will vary: some inputs are more likely than others. 
These differences are captured by the operational profile (OP)

## Reliability modelling framework
- reliability is expressed as the probability of not failing on a randomly chosen input $d_r ∈ D$.
Let F be a random variable (r.v.) that represents this probability. 
The service reliability then can be expressed via the r.v. R = 1 − F.
- 

# Opertional Reliability Assesment 

Through testing

### Classification with 10 classes Reliability
From : https://www.kaggle.com/code/kooaslansefat/evidential-deep-learning-reliability
```
y_pred1 = model.predict(x_test)
y_pred =  np.argmax(y_pred1,axis=1)

# Separating Wrong Responses of the CNN Classifier
X_test_wrong, y_test_wrong = x_test[np.where(y_test != y_pred)], y_test[np.where(y_test != y_pred)]

# Separating Correct Responses of the CNN Classifier
X_test_correct, y_test_correct = x_test[np.where(y_test == y_pred)], y_test[np.where(y_test == y_pred)]

r = 0
N = 0
E_F = np.zeros(10)

for ii in range(10):
    X_test_wrong_i, y_test_wrong_i = X_test_wrong[np.where(y_test_wrong == ii+1)], y_test_wrong[np.where(y_test_wrong == ii+1)]
    X_test_correct_i, y_test_correct_i = x_test[np.where(y_test_correct == ii+1)], y_test[np.where(y_test_correct == ii+1)]
    r = X_test_wrong_i.shape[0]
    N = X_test_wrong_i.shape[0] + X_test_correct_i.shape[0] 
    
    E_F[ii] = (1 + r)/(1+1+N)
    
Reliability = 1 - 0.1*(sum(E_F))

print(Reliability)
```

Its bayesian rule of posterior for the beta distribution. Where the posterior is just adding the number of positives to alpha and negatives to beta .
It then multiplied to the dirchlet prior value(0.1).
Then the values of summed.

## Big Idea 

The main idea of the paper [3] is the **Operational Profile** , which basically separates the input data space into multiple subset.
The subset can be additional to the classes in the dataset.

# References

[1] Kapur, Kailash C., and Michael Pecht. "Reliability engineering." (2014).
[2] Saria, Suchi, and Adarsh Subbaswamy. "Tutorial: safe and reliable machine learning." arXiv preprint arXiv:1904.07204 (2019).
[3] Pietrantuono, Roberto et al. “Reliability assessment of service-based software under operational profile uncertainty.”
Reliab. Eng. Syst. Saf. 204 (2020): 107193.
