#!/usr/bin/env groovy
@Library('tailor-meta@apt-fix-refactor')_
tailorTestPipeline(
  rosdistro: 'ros1',
  // Release track to test branch against.
  release_track: 'hotdog',
  // OS distributions to test.
  distributions: ['xenial', 'bionic'],
  // Bundle flavour to test against.
  flavour: 'dev',
  // Branch of tailor_meta to build against
  meta_branch: 'apt-fix-refactor',
  // Master branch of this repo, to determine whether to automatically trigger builds
  master_branch: 'devel',
  // Docker registry where test image is stored
  docker_registry: 'https://084758475884.dkr.ecr.us-east-1.amazonaws.com/locus-tailor'
)