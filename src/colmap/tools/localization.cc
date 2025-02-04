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
// #include "colmap/scene/reconstruction_io.h"
#include "colmap/util/logging.h"
#include "colmap/util/file.h"
#include "colmap/util/misc.h"
#include "colmap/util/timer.h"
#include "colmap/sfm/incremental_mapper.h"
#include "colmap/scene/database_cache.h"
// #include "colmap/exe/sfm.h"
#include "colmap/controllers/incremental_pipeline.h"


#include <filesystem>
#include <iostream>
#include <fstream>
#include <vector>
#include <algorithm>
#include <random>

using namespace colmap;

// adapted from exe/image.cc's RunImageRegistrator
int RunImageRegistratorImpl(std::string input_path, std::string output_path, std::string database_path) {
//   std::string input_path;
//   std::string output_path;

//   OptionManager options;
//   options.AddDatabaseOptions();
//   options.AddRequiredOption("input_path", &input_path);
//   options.AddRequiredOption("output_path", &output_path);
//   options.AddMapperOptions();
//   options.Parse(argc, argv);

  const IncrementalPipelineOptions pipe_options = IncrementalPipelineOptions();

  if (!ExistsDir(input_path)) {
    LOG(ERROR) << "`input_path` is not a directory";
    return EXIT_FAILURE;
  }

  if (!ExistsDir(output_path)) {
    LOG(ERROR) << "`output_path` is not a directory";
    return EXIT_FAILURE;
  }

  PrintHeading1("Loading database");

  std::shared_ptr<DatabaseCache> database_cache;

  {
    Timer timer;
    timer.Start();
    const size_t min_num_matches =
        static_cast<size_t>(pipe_options.min_num_matches);
    
    //TODO horiz: are the query intrinsics now already in the database?
    //TODO horiz: do we need to choose the db images for image_names? -> probably not because it's good if we have all info also from queries in database cache so that we have query detections and matches.
    database_cache = DatabaseCache::Create(Database(database_path),
                                           min_num_matches,
                                           pipe_options.ignore_watermarks,
                                           pipe_options.image_names); // TODO horiz: here we can choose which images to reconstruct.
    timer.PrintMinutes();
  }

  auto reconstruction = std::make_shared<Reconstruction>();
  reconstruction->Read(input_path);

  //TODO horiz: check what images are in the reconstruction now and with what properties set.
  IncrementalMapper mapper(database_cache);
  //TODO horiz: Within  mapper.BeginReconstruction, they do reconstruction.Load(database_path)
  mapper.BeginReconstruction(reconstruction);

  const auto mapper_options = pipe_options.Mapper();

  for (const auto& image : reconstruction->Images()) {
    if (image.second.HasPose()) {
      continue;
    }

    PrintHeading1("Registering image #" + std::to_string(image.first) + " (" +
                  std::to_string(reconstruction->NumRegImages() + 1) + ")");

    LOG(INFO) << "\n=> Image sees "
              << mapper.ObservationManager().NumVisiblePoints3D(image.first)
              << " / "
              << mapper.ObservationManager().NumObservations(image.first)
              << " points";

    mapper.RegisterNextImage(mapper_options, image.first);
  }

  mapper.EndReconstruction(/*discard=*/false);

  reconstruction->Write(output_path);
  reconstruction->WriteText(output_path);

  return EXIT_SUCCESS;
}


// Simple example that reads and writes a reconstruction.
int main(int argc, char** argv) {
  InitializeGlog(argv);
  FLAGS_v = 2;
  FLAGS_logtostderr = true;

  // std::string input_path;
  // std::string output_path;

  // colmap::OptionManager options;
  // options.AddRequiredOption("input_path", &input_path);
  // options.AddRequiredOption("output_path", &output_path);
  // options.Parse(argc, argv);

  // For scannet
  // std::string scene_dir = "/local/home/vfrawa/data/ScanNet/scans/scene0191_00";
  // std::string empty_colmap_model_with_known_poses_path = scene_dir + "/formats/empty_colmap_model_known_poses";
  // std::string database_path = scene_dir + "/outputs_hloc/sfm_superpoint+superglue/database.db";
  // std::string images_dir = scene_dir + "/sensorstream";
  // std::string reference_sfm = scene_dir + "/outputs_colmap/test2/sfm_superpoint+superglue";

  //for hypersim
  std::string scene_dir = "/local/home/vfrawa/data/hypersim";
  std::string existing_db_colmap_model_path = scene_dir + "/outputs_hloc/test_db_query_split/sfm_superpoint+superglue";
  std::string database_path = scene_dir + "/outputs_hloc/test_db_query_split/sfm_superpoint+superglue/database.db";
//   std::string database_path = scene_dir + "/db.db";
  std::string images_dir = scene_dir + "/images";
  std::string reference_sfm = scene_dir + "/outputs_colmap/test2_db_query_split/sfm_superpoint+superglue";

  // Create the directories
  try {
      std::filesystem::create_directories(reference_sfm);
      std::cout << "Directory created successfully: " << reference_sfm << std::endl;
  } catch (const std::filesystem::filesystem_error& e) {
      std::cerr << "Error creating directories: " << e.what() << std::endl;
  }

  RunImageRegistratorImpl(existing_db_colmap_model_path, reference_sfm, database_path);

  // Create a shared pointer to the reconstruction
//   std::shared_ptr<colmap::Reconstruction> empty_colmap_model_with_known_poses_ptr = 
//   std::make_shared<colmap::Reconstruction>(empty_colmap_model_with_known_poses);

//   colmap::IncrementalPipelineOptions options = colmap::IncrementalPipelineOptions();
//   colmap::RunPointTriangulatorImpl(empty_colmap_model_with_known_poses_ptr, database_path, images_dir, reference_sfm, options, false, false, do_global_refinement);

//   colmap::Reconstruction reference_sfm_model;
//   reference_sfm_model.Read(reference_sfm);
//   reference_sfm_model.WriteText(reference_sfm);

//   colmap::ExportPLY(reference_sfm_model, reference_sfm + "/horizprojected3dpoints.ply", true);
//   colmap::ExportPLY(reference_sfm_model, reference_sfm + "/3dpoints.ply", false);

  return EXIT_SUCCESS;
}
