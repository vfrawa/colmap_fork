// Copyright (c) 2023, ETH Zurich and UNC Chapel Hill.
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//
//     * Neither the name of ETH Zurich and UNC Chapel Hill nor the names of
//       its contributors may be used to endorse or promote products derived
//       from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

//#include "colmap/controllers/option_manager.h"
#include "colmap/scene/reconstruction.h"
//#include "colmap/util/logging.h"
#include "colmap/exe/sfm.h"
#include "colmap/controllers/incremental_pipeline.h"
#include <filesystem>
#include <iostream>

// Simple example that reads and writes a reconstruction.
//int main(int argc, char** argv) {
int main() {
  //colmap::InitializeGlog(argv);

  // std::string input_path;
  // std::string output_path;

  // colmap::OptionManager options;
  // options.AddRequiredOption("input_path", &input_path);
  // options.AddRequiredOption("output_path", &output_path);
  // options.Parse(argc, argv);

  std::string scene_dir = "/local/home/vfrawa/data/ScanNet/scans/scene0191_00";

  std::string empty_colmap_model_with_known_poses_path = scene_dir + "/formats_shortpath/empty_colmap_model_known_poses";
  std::string database_path = scene_dir + "/outputs_hloc/sfm_superpoint+superglue/database.db";
  std::string images_dir = scene_dir + "/sensorstream";
  std::string reference_sfm = scene_dir + "/outputs_colmap/sfm_superpoint+superglue_direct_colmap/";

  // Create the directories
  try {
      std::filesystem::create_directories(reference_sfm);
      std::cout << "Directory created successfully: " << reference_sfm << std::endl;
  } catch (const std::filesystem::filesystem_error& e) {
      std::cerr << "Error creating directories: " << e.what() << std::endl;
  }
  colmap::Reconstruction empty_colmap_model_with_known_poses;
  empty_colmap_model_with_known_poses.Read(empty_colmap_model_with_known_poses_path);
  //empty_colmap_model_with_known_poses.Write(output_path);

// void RunPointTriangulatorImpl(
//     const std::shared_ptr<Reconstruction>& reconstruction,
//     const std::string& database_path,
//     const std::string& image_path,
//     const std::string& output_path,
//     const IncrementalPipelineOptions& options,
//     const bool clear_points,
//     const bool refine_intrinsics)

  // Create a shared pointer to the reconstruction
  std::shared_ptr<colmap::Reconstruction> empty_colmap_model_with_known_poses_ptr = 
  std::make_shared<colmap::Reconstruction>(empty_colmap_model_with_known_poses);

  colmap::IncrementalPipelineOptions options = colmap::IncrementalPipelineOptions();
  colmap::RunPointTriangulatorImpl(empty_colmap_model_with_known_poses_ptr, database_path, images_dir, reference_sfm, options, false, false); //TODO maybe make clear_points true?

  return EXIT_SUCCESS;
}
