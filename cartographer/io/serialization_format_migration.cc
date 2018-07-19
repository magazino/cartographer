/*
 * Copyright 2018 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "cartographer/io/serialization_format_migration.h"

#include <string>
#include <unordered_map>
#include <vector>

#include "cartographer/mapping/2d/submap_2d.h"
#include "cartographer/mapping/proto/internal/legacy_probability_grid.pb.h"
#include "cartographer/mapping/proto/internal/legacy_serialized_data.pb.h"
#include "cartographer/mapping/proto/internal/legacy_submap.pb.h"
#include "cartographer/mapping/proto/trajectory_builder_options.pb.h"
#include "glog/logging.h"

namespace cartographer {
namespace io {
namespace {

using mapping::proto::SerializedData;
using ProtoMap = std::unordered_map<int, std::vector<SerializedData>>;

bool ReadPoseGraph(cartographer::io::ProtoStreamReaderInterface* const input,
                   ProtoMap* proto_map) {
  auto& pose_graph_vec = (*proto_map)[SerializedData::kPoseGraph];
  pose_graph_vec.emplace_back();
  return input->ReadProto(pose_graph_vec.back().mutable_pose_graph());
}

bool ReadBuilderOptions(
    cartographer::io::ProtoStreamReaderInterface* const input,
    ProtoMap* proto_map) {
  auto& options_vec =
      (*proto_map)[SerializedData::kAllTrajectoryBuilderOptions];
  options_vec.emplace_back();
  return input->ReadProto(
      options_vec.back().mutable_all_trajectory_builder_options());
}

// Checks if the proto is in the old "probability-grid-only" 2D submap format
// that was used before the generalized grid field was introduced.
bool HasLegacyProbabilityGrid2d(const mapping::proto::Submap& submap) {
  if (submap.has_submap_2d()) {
    if (!submap.submap_2d().grid().has_probability_grid_2d() &&
        !submap.submap_2d().grid().has_tsdf_2d()) {
      return true;
    }
  }
  return false;
}

mapping::proto::Submap MaybeMigrateLegacySubmap2d(
    const mapping::proto::Submap& submap_in) {
  if (HasLegacyProbabilityGrid2d(submap_in)) {
    mapping::proto::Submap2D submap_2d;
    *submap_2d.mutable_local_pose() = submap_in.submap_2d().local_pose();
    submap_2d.set_num_range_data(submap_in.submap_2d().num_range_data());
    submap_2d.set_finished(submap_in.submap_2d().finished());

    // grid() is a disguised legacy probability_grid() - migrate it.
    // We can't directly convert it because the proto field numbers were changed
    // in a non-compatible way :( ---> known_cells_box was 8, now 3 .... meh
    std::string probability_grid_binary;
    submap_in.submap_2d().grid().SerializeToString(&probability_grid_binary);
    mapping::proto::LegacyProbabilityGrid tmp;
    tmp.ParseFromString(probability_grid_binary);
    *submap_2d.mutable_grid()->mutable_limits() = tmp.limits();
    submap_2d.mutable_grid()->mutable_cells()->CopyFrom(tmp.cells());

    // TODO: these make problems:

    *submap_2d.mutable_grid()->mutable_known_cells_box() =
        dynamic_cast<mapping::proto::Grid2D_CellBox&>(
            tmp.mutable_known_cells_box());
    submap_2d.mutable_grid()->mutable_probability_grid_2d();

    mapping::proto::Submap submap_out;
    *submap_out.mutable_submap_2d() = submap_2d;
    *submap_out.mutable_submap_id() = submap_in.submap_id();
    return submap_out;
  } else {
    // Already in newer format - nothing to do.
    return submap_in;
  }
}

bool DeserializeNext(cartographer::io::ProtoStreamReaderInterface* const input,
                     ProtoMap* proto_map) {
  mapping::proto::LegacySerializedData legacy_data;
  if (!input->ReadProto(&legacy_data)) return false;

  if (legacy_data.has_submap()) {
    LOG_FIRST_N(INFO, 1) << "Migrating submap data.";
    auto& output_vector = (*proto_map)[SerializedData::kSubmapFieldNumber];
    output_vector.emplace_back();
    *output_vector.back().mutable_submap() =
        MaybeMigrateLegacySubmap2d(legacy_data.submap());
  }
  if (legacy_data.has_node()) {
    LOG_FIRST_N(INFO, 1) << "Migrating node data.";
    auto& output_vector = (*proto_map)[SerializedData::kNodeFieldNumber];
    output_vector.emplace_back();
    *output_vector.back().mutable_node() = legacy_data.node();
  }
  if (legacy_data.has_trajectory_data()) {
    LOG_FIRST_N(INFO, 1) << "Migrating trajectory data.";
    auto& output_vector =
        (*proto_map)[SerializedData::kTrajectoryDataFieldNumber];
    output_vector.emplace_back();
    *output_vector.back().mutable_trajectory_data() =
        legacy_data.trajectory_data();
  }
  if (legacy_data.has_imu_data()) {
    LOG_FIRST_N(INFO, 1) << "Migrating IMU data.";
    auto& output_vector = (*proto_map)[SerializedData::kImuDataFieldNumber];
    output_vector.emplace_back();
    *output_vector.back().mutable_imu_data() = legacy_data.imu_data();
  }
  if (legacy_data.has_odometry_data()) {
    LOG_FIRST_N(INFO, 1) << "Migrating odometry data.";
    auto& output_vector = (*proto_map)[SerializedData::kOdometryData];
    output_vector.emplace_back();
    *output_vector.back().mutable_odometry_data() = legacy_data.odometry_data();
  }
  if (legacy_data.has_fixed_frame_pose_data()) {
    LOG_FIRST_N(INFO, 1) << "Migrating fixed frame pose data.";
    auto& output_vector =
        (*proto_map)[SerializedData::kFixedFramePoseDataFieldNumber];
    output_vector.emplace_back();
    *output_vector.back().mutable_fixed_frame_pose_data() =
        legacy_data.fixed_frame_pose_data();
  }
  if (legacy_data.has_landmark_data()) {
    LOG_FIRST_N(INFO, 1) << "Migrating landmark data.";
    auto& output_vector =
        (*proto_map)[SerializedData::kLandmarkDataFieldNumber];
    output_vector.emplace_back();
    *output_vector.back().mutable_landmark_data() = legacy_data.landmark_data();
  }
  return true;
}

ProtoMap ParseLegacyData(
    cartographer::io::ProtoStreamReaderInterface* const input) {
  ProtoMap proto_map;
  CHECK(ReadPoseGraph(input, &proto_map))
      << "Input stream seems to differ from original stream format. Could "
         "not "
         "read PoseGraph as first message.";
  CHECK(ReadBuilderOptions(input, &proto_map))
      << "Input stream seems to differ from original stream format. Could "
         "not "
         "read AllTrajectoryBuilderOptions as second message.";
  do {
  } while (DeserializeNext(input, &proto_map));
  return proto_map;
}

mapping::proto::SerializationHeader CreateSerializationHeader() {
  constexpr uint32_t kVersion1 = 1;
  mapping::proto::SerializationHeader header;
  header.set_format_version(kVersion1);
  return header;
}

void SerializeToVersion1Format(
    const ProtoMap& deserialized_data,
    cartographer::io::ProtoStreamWriterInterface* const output) {
  const std::vector<int> kFieldSerializationOrder = {
      SerializedData::kPoseGraphFieldNumber,
      SerializedData::kAllTrajectoryBuilderOptionsFieldNumber,
      SerializedData::kSubmapFieldNumber,
      SerializedData::kNodeFieldNumber,
      SerializedData::kTrajectoryDataFieldNumber,
      SerializedData::kImuDataFieldNumber,
      SerializedData::kOdometryDataFieldNumber,
      SerializedData::kFixedFramePoseDataFieldNumber,
      SerializedData::kLandmarkDataFieldNumber};

  output->WriteProto(CreateSerializationHeader());
  for (auto field_index : kFieldSerializationOrder) {
    const auto proto_vector_it = deserialized_data.find(field_index);
    if (proto_vector_it == deserialized_data.end()) continue;
    for (const auto& proto : proto_vector_it->second) {
      output->WriteProto(proto);
    }
  }
}
}  // namespace

void MigrateStreamFormatToVersion1(
    cartographer::io::ProtoStreamReaderInterface* const input,
    cartographer::io::ProtoStreamWriterInterface* const output) {
  SerializeToVersion1Format(ParseLegacyData(input), output);
}

}  // namespace io
}  // namespace cartographer
