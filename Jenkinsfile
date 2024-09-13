#!/usr/bin/env groovy
@Library('tailor-meta@0.1.24')_
tailorTestPipeline(
  // Name of job that generated this test definition.
  rosdistro_job: '/ci/rosdistro/release%2F24.1',
  // Distribution name
  rosdistro_name: 'ros2',
  // Release track to test branch against.
  release_track: '24.1',
  // Release label to pull test images from.
  release_label: '24.1-rc',
  // OS distributions to test.
  distributions: ['jammy'],
  // Version of tailor_meta to build against
  tailor_meta: '0.1.24',
  // Master or release branch associated with this track
  source_branch: 'iron',
  // Docker registry where test image is stored
  docker_registry: 'https://084758475884.dkr.ecr.us-east-1.amazonaws.com/locus-tailor'
)