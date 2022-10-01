#!/usr/bin/env groovy
@Library('tailor-meta@0.1.18')_
tailorTestPipeline(
  // Name of job that generated this test definition.
  rosdistro_job: '/ci/rosdistro/release%2F22.3',
  // Distribution name
  rosdistro_name: 'ros1',
  // Release track to test branch against.
  release_track: '22.3',
  // Release label to pull test images from.
  release_label: '22.3-rc',
  // OS distributions to test.
  distributions: ['focal'],
  // Version of tailor_meta to build against
  tailor_meta: '0.1.18',
  // Master or release branch associated with this track
  source_branch: 'release/22',
  // Docker registry where test image is stored
  docker_registry: 'https://084758475884.dkr.ecr.us-east-1.amazonaws.com/locus-tailor'
)