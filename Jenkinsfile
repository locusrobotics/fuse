#!/usr/bin/env groovy
@Library('tailor-meta@0.1.4')_
tailorTestPipeline(
  // Name of job that generated this test definition.
  rosdistro_job: '/ci/rosdistro/master',
  // Distribution name
  rosdistro_name: 'ros1',
  // Release track to test branch against.
  release_track: 'hotdog',
  // OS distributions to test.
  distributions: ['xenial', 'bionic'],
  // Bundle flavour to test against.
  flavour: 'dev',
  // Branch of tailor_meta to build against
  tailor_meta_branch: '0.1.4',
  // Master branch of this repo, to determine whether to automatically trigger builds
  repo_main_branch: 'devel',
  // Docker registry where test image is stored
  docker_registry: 'https://084758475884.dkr.ecr.us-east-1.amazonaws.com/locus-tailor'
)