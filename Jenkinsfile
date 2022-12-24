#!/usr/bin/env groovy
@Library('tailor-meta@0.1.20')_
tailorTestPipeline(
  // Name of job that generated this test definition.
  rosdistro_job: '/ci/rosdistro/release%2F23',
  // Distribution name
  rosdistro_name: 'ros1',
  // Release track to test branch against.
  release_track: '23',
  // Release label to pull test images from.
  release_label: '23-rc',
  // OS distributions to test.
  distributions: ['focal'],
  // Version of tailor_meta to build against
  tailor_meta: '0.1.20',
  // Master or release branch associated with this track
  source_branch: 'release/23',
  // Docker registry where test image is stored
  docker_registry: 'https://084758475884.dkr.ecr.us-east-1.amazonaws.com/locus-tailor'
)