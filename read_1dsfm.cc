#include <boost/progress.hpp>

#include "base/camera_models.h"
#include "base/database.h"
#include "base/pose.h"
#include "util/bitmap.h"
#include "util/logging.h"
#include "util/misc.h"
#include "util/option_manager.h"

using namespace colmap;

class Input1DSfM {
 public:
  Input1DSfM(const std::string& dataset_directory,
             const std::string& image_path, const std::string& database_path);

  bool Read();

 private:
  struct ListFile {
    std::string image_name;
    bool has_prior_focal_lenth = false;
    double prior_focal_length = 0.0;
  };

  struct EpipolarGeometry {
    image_t image_id1;
    image_t image_id2;

    // Transformation from camera 1 to camera 2.
    Eigen::Vector4d qvec12;
    Eigen::Vector3d tvec12;
  };

  struct Feature {
    uint32_t id;
    Eigen::Vector2d xy;
    Eigen::Vector3i color;
  };

  struct Track {
    std::vector<std::pair<image_t, uint32_t>> elements;
  };

  bool ReadListsFile();
  bool ReadEGs();
  bool ReadCoords();
  bool ReadTracks();

  bool WriteImages();
  bool WriteMatches();

  bool ToDatabase();

  bool ReadCoordsHeaderLine(const std::string& line, uint32_t* image_id,
                            uint32_t* num_features);

  const std::string& dataset_directory_;
  const std::string& image_path_;

  Database database_;

  std::vector<ListFile> list_file_;
  std::vector<EpipolarGeometry> epipolar_geometries_;
  std::unordered_map<image_t, std::vector<Feature>> coords_;
  std::vector<Track> tracks_;
};

void Read1DSfM(int argc, char** argv) {
  std::string dataset_directory;

  OptionManager options;
  options.AddDatabaseOptions();
  options.AddImageOptions();
  options.AddRequiredOption("dataset_directory", &dataset_directory,
                            "1DSfM dataset directory");
  options.Parse(argc, argv);

  Input1DSfM reader(dataset_directory, *options.image_path.get(),
                    *options.database_path.get());

  if (!reader.Read()) {
    std::cerr << "ERROR: Failed to read 1DSfM dataset" << std::endl;
    return;
  }
}

int main(int argc, char** argv) {
  InitializeGlog(argv);
  Read1DSfM(argc, argv);
}

Input1DSfM::Input1DSfM(const std::string& dataset_directory,
                       const std::string& image_path,
                       const std::string& database_path)
    : dataset_directory_(dataset_directory),
      image_path_(image_path),
      database_(database_path) {
  CHECK(ExistsDir(dataset_directory_)) << "ERROR: directory doesn't exist";
}

bool Input1DSfM::Read() {
  bool success = true;

  PrintHeading1("Reading 1DSfM dataset");
  success = success && ReadListsFile();
  success = success && ReadEGs();
  success = success && ReadCoords();
  success = success && ReadTracks();

  PrintHeading1("Writing 1DSfM dataset into COLMAP database");
  success = success && ToDatabase();

  return success;
}

bool Input1DSfM::ReadListsFile() {
  PrintHeading2("Reading the list file");

  const std::string filename = dataset_directory_ + "/list.txt";

  list_file_.clear();

  std::ifstream file(filename);
  CHECK(file.is_open()) << "ERROR: Can't read file `list.txt`";

  std::string line;
  while (std::getline(file, line)) {
    ListFile list_item;
    StringTrim(&line);

    std::vector<std::string> strs = StringSplit(line, " ");
    auto tmp = StringSplit(strs[0], "/");
    list_item.image_name = tmp[tmp.size() - 1];
    if (strs.size() == 1) {
      list_item.has_prior_focal_lenth = false;
    } else {
      list_item.has_prior_focal_lenth = true;
      list_item.prior_focal_length = std::stold(strs[strs.size() - 1]);
    }
    list_file_.emplace_back(list_item);
  }

  return true;
}

bool Input1DSfM::ReadEGs() {
  PrintHeading2("Reading epipolar geometries");

  const std::string filename = dataset_directory_ + "/EGs.txt";

  epipolar_geometries_.clear();

  std::ifstream file(filename);
  CHECK(file.is_open()) << "ERROR: Can't read file `EGs.txt`";

  std::string line;
  while (std::getline(file, line)) {
    EpipolarGeometry eg;

    StringTrim(&line);
    std::stringstream line_stream(line);

    int view_id1, view_id2;
    line_stream >> view_id1;
    line_stream >> view_id2;

    eg.image_id1 = static_cast<image_t>(view_id1);
    eg.image_id2 = static_cast<image_t>(view_id2);

    // 1DSfM defines transformation from camera 2 to camera 1, we want 1 to 2.
    Eigen::Matrix3d R_2_to_1;
    for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 3; j++) {
        line_stream >> R_2_to_1(i, j);
      }
    }
    Eigen::Vector3d t_2_to_1;
    for (int i = 0; i < 3; i++) {
      line_stream >> t_2_to_1(i);
    }
    Eigen::Vector4d qvec21 = RotationMatrixToQuaternion(R_2_to_1);
    NormalizeQuaternion(qvec21);
    InvertPose(qvec21, t_2_to_1, &eg.qvec12, &eg.tvec12);
    epipolar_geometries_.emplace_back(eg);
  }
  return true;
}

bool Input1DSfM::ReadCoords() {
  PrintHeading2("Reading feature coordinates");

  const std::string filename = dataset_directory_ + "/coords.txt";

  coords_.clear();

  std::ifstream file(filename);
  CHECK(file.is_open()) << "ERROR: Can't read file `coords.txt`";

  std::string line;
  while (std::getline(file, line)) {
    uint32_t image_id;
    uint32_t num_features;
    ReadCoordsHeaderLine(line, &image_id, &num_features);

    std::vector<Feature> features;
    features.reserve(num_features);
    for (size_t i = 0; i < num_features; i++) {
      Feature feature;
      std::getline(file, line);
      StringTrim(&line);
      std::stringstream line_stream(line);
      int tmp;
      line_stream >> feature.id;
      line_stream >> feature.xy(0) >> feature.xy(1);
      line_stream >> tmp >> tmp;
      line_stream >> feature.color(0) >> feature.color(1) >> feature.color(2);
      features.push_back(feature);
    }
    coords_.emplace(image_id, features);
  }
  return true;
}

bool Input1DSfM::ReadTracks() {
  PrintHeading2("Reading tracks");

  const std::string filename = dataset_directory_ + "/tracks.txt";

  std::ifstream file(filename);
  CHECK(file.is_open()) << "ERROR: Can't read file `tracks.txt`";

  // Read number of tracks.
  std::string line;
  std::getline(file, line);
  StringTrim(&line);
  size_t num_tracks = std::stoll(line);

  tracks_.reserve(num_tracks);

  while (std::getline(file, line)) {
    StringTrim(&line);
    std::stringstream line_stream(line);

    size_t num_features;
    line_stream >> num_features;
    Track track;
    track.elements.reserve(num_features);
    for (size_t i = 0; i < num_features; i++) {
      image_t image_id;
      uint32_t feature_id;
      line_stream >> image_id;
      line_stream >> feature_id;
      track.elements.emplace_back(image_id, feature_id);
    }
    tracks_.push_back(track);
  }
  return true;
}

bool Input1DSfM::WriteImages() {
  PrintHeading2("Writing images into database");

  for (size_t i = 0; i < list_file_.size(); i++) {
    const ListFile& item = list_file_[i];
    image_t image_id = i;

    if (coords_.find(image_id) == coords_.end()) {
      continue;
    }

    Image image;
    image.SetImageId(image_id);
    image.SetName(item.image_name);

    const std::string& image_filepath = JoinPaths(image_path_, item.image_name);
    Bitmap bitmap;
    CHECK(bitmap.Read(image_filepath, false))
        << "Read image " << item.image_name
        << " , please check if the image exists";

    // Default focal length factor
    double default_focal_length_factor = 1.2;
    double focal_length;
    if (item.has_prior_focal_lenth) {
      focal_length = item.prior_focal_length;
    } else {
      focal_length = default_focal_length_factor *
                     std::max(bitmap.Width(), bitmap.Height());
    }

    // Setup the camera.
    Camera camera;
    camera.SetCameraId(image_id);
    camera.SetPriorFocalLength(item.has_prior_focal_lenth);
    camera.InitializeWithName("SIMPLE_RADIAL", focal_length, bitmap.Width(),
                              bitmap.Height());

    image.SetCameraId(camera.CameraId());
    database_.WriteCamera(camera, true);
    database_.WriteImage(image, true);

    // Get feature points;
    const std::vector<Feature>& feature_points = coords_[image_id];
    FeatureKeypoints keypoints;
    keypoints.reserve(feature_points.size());
    for (const auto& point : feature_points) {
      FeatureKeypoint feature(point.xy(0), point.xy(1));
      keypoints.push_back(feature);
    }
    database_.WriteKeypoints(image_id, keypoints);
    FeatureDescriptors fake_descriptors;
    fake_descriptors.resize(keypoints.size(), 128);
    database_.WriteDescriptors(image_id, fake_descriptors);
  }
  return true;
}

bool Input1DSfM::WriteMatches() {
  PrintHeading2("Writing two-view geometries into database");

  boost::progress_display progress_bar(epipolar_geometries_.size());

  for (size_t pair_idx = 0; pair_idx < epipolar_geometries_.size();
       pair_idx++) {
    const EpipolarGeometry& pair_item = epipolar_geometries_[pair_idx];

    image_t image_id1 = pair_item.image_id1;
    image_t image_id2 = pair_item.image_id2;

    if (!database_.ExistsImage(pair_item.image_id1) ||
        !database_.ExistsImage(pair_item.image_id2)) {
      continue;
    }

    FeatureMatches matches;

    // Iterate over the tracks and find out all the feature matches.
    for (size_t track_idx = 0; track_idx < tracks_.size(); track_idx++) {
      const Track& track = tracks_[track_idx];
      FeatureMatch match;

      for (const auto& pair : track.elements) {
        if (pair.first == image_id1) {
          match.point2D_idx1 = pair.second;
        }
        if (pair.first == image_id2) {
          match.point2D_idx2 = pair.second;
        }
      }

      if (match.point2D_idx1 == kInvalidPoint2DIdx ||
          match.point2D_idx2 == kInvalidPoint2DIdx) {
        continue;
      }
      matches.emplace_back(match);
    }

    if (matches.size() == 0) {
      std::cout << "WARN: No feature matches found in this image pair"
                << std::endl;
    }
    // Create Two-view geometry
    TwoViewGeometry two_view_geometry;
    two_view_geometry.qvec = pair_item.qvec12;
    two_view_geometry.tvec = pair_item.tvec12;
    two_view_geometry.config = TwoViewGeometry::UNCALIBRATED;
    two_view_geometry.inlier_matches = matches;

    database_.WriteMatches(image_id1, image_id2, matches);
    database_.WriteTwoViewGeometry(image_id1, image_id2, two_view_geometry);
    ++progress_bar;
  }
  return true;
}

bool Input1DSfM::ToDatabase() {
  DatabaseTransaction database_transaction(&database_);

  bool success = true;

  success = success && WriteImages();
  success = success && WriteMatches();

  return success;
}

bool Input1DSfM::ReadCoordsHeaderLine(const std::string& line,
                                      uint32_t* image_id,
                                      uint32_t* num_features) {
  float principal_point_x, principal_point_y, focal_length;
  char name[256];
  sscanf(line.c_str(),
         "#index = %d, name = %s keys = %d, px = %f, py = %f, focal = %f",
         image_id, name, num_features, &principal_point_x, &principal_point_y,
         &focal_length);
  return true;
}