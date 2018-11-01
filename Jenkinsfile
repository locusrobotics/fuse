#!/usr/bin/env groovy
@Library('tailor-meta@0.0.3')_
tailorTestPipeline(
  rosdistro: 'ros1',
  // Release track to test branch against.
  release_track: 'hotdog',
  // OS distributions to test.
  distributions: ['xenial'],
  // Bundle flavour to test against.
  flavour: 'dev',
  // Branch of tailor_meta to build against
  meta_branch: '0.0.3',
  // Master branch of this repo, to determine whether to automatically trigger builds
  master_branch: 'devel'
)