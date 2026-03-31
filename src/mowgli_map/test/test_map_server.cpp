// Copyright (C) 2024 Cedric <cedric@mowgli.dev>
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "mowgli_map/map_server_node.hpp"
#include "mowgli_map/map_types.hpp"
#include <gtest/gtest.h>

// ─────────────────────────────────────────────────────────────────────────────
// Test fixture — creates a MapServerNode with a small 10×10 m map
// ─────────────────────────────────────────────────────────────────────────────

class MapServerTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    rclcpp::init(0, nullptr);
  }

  static void TearDownTestSuite()
  {
    rclcpp::shutdown();
  }

  void SetUp() override
  {
    rclcpp::NodeOptions opts;
    opts.append_parameter_override("resolution", 0.1);
    opts.append_parameter_override("map_size_x", 10.0);
    opts.append_parameter_override("map_size_y", 10.0);
    opts.append_parameter_override("map_frame", "map");
    opts.append_parameter_override("decay_rate_per_hour", 3600.0);  // 1.0 / sec for easy maths
    opts.append_parameter_override("mower_width", 0.2);
    opts.append_parameter_override("map_file_path", "");
    opts.append_parameter_override("publish_rate", 1.0);

    node_ = std::make_shared<mowgli_map::MapServerNode>(opts);
  }

  void TearDown() override
  {
    node_.reset();
  }

  std::shared_ptr<mowgli_map::MapServerNode> node_;
};

// ─────────────────────────────────────────────────────────────────────────────
// Test 1 — grid_map creation with correct layers
// ─────────────────────────────────────────────────────────────────────────────

TEST_F(MapServerTest, GridMapHasAllRequiredLayers)
{
  std::lock_guard<std::mutex> lock(node_->map_mutex());
  const auto& m = node_->map();

  EXPECT_TRUE(m.exists(std::string(mowgli_map::layers::OCCUPANCY)));
  EXPECT_TRUE(m.exists(std::string(mowgli_map::layers::CLASSIFICATION)));
  EXPECT_TRUE(m.exists(std::string(mowgli_map::layers::MOW_PROGRESS)));
  EXPECT_TRUE(m.exists(std::string(mowgli_map::layers::CONFIDENCE)));
}

TEST_F(MapServerTest, GridMapGeometryIsCorrect)
{
  std::lock_guard<std::mutex> lock(node_->map_mutex());
  const auto& m = node_->map();

  // 10 m / 0.1 m resolution = 100 cells per axis
  EXPECT_EQ(m.getSize()(0), 100);
  EXPECT_EQ(m.getSize()(1), 100);

  EXPECT_DOUBLE_EQ(m.getResolution(), 0.1);
  EXPECT_EQ(m.getFrameId(), "map");
}

TEST_F(MapServerTest, GridMapLayersInitialisedToDefaults)
{
  std::lock_guard<std::mutex> lock(node_->map_mutex());
  const auto& m = node_->map();

  // All occupancy cells must be 0.0 (free)
  const auto& occ = m[std::string(mowgli_map::layers::OCCUPANCY)];
  EXPECT_FLOAT_EQ(occ.minCoeff(), 0.0F);
  EXPECT_FLOAT_EQ(occ.maxCoeff(), 0.0F);

  // All mow_progress cells must be 0.0 (unmowed)
  const auto& prog = m[std::string(mowgli_map::layers::MOW_PROGRESS)];
  EXPECT_FLOAT_EQ(prog.minCoeff(), 0.0F);
  EXPECT_FLOAT_EQ(prog.maxCoeff(), 0.0F);

  // All confidence cells must be 0.0
  const auto& conf = m[std::string(mowgli_map::layers::CONFIDENCE)];
  EXPECT_FLOAT_EQ(conf.minCoeff(), 0.0F);
  EXPECT_FLOAT_EQ(conf.maxCoeff(), 0.0F);
}

// ─────────────────────────────────────────────────────────────────────────────
// Test 2 — mow_progress decay calculation
// ─────────────────────────────────────────────────────────────────────────────

TEST_F(MapServerTest, MowProgressDecaysOverTime)
{
  // First mark the centre cell as fully mowed
  node_->mark_mowed(0.0, 0.0);

  {
    std::lock_guard<std::mutex> lock(node_->map_mutex());
    const auto& prog = node_->map()[std::string(mowgli_map::layers::MOW_PROGRESS)];
    ASSERT_GT(prog.maxCoeff(), 0.0F) << "At least one cell should be mowed before testing decay";
  }

  // decay_rate_per_hour = 3600.0, so after 1 second the decay is 1.0 per cell.
  // After 0.5 s the freshly mowed cells (1.0) should decay to ~0.5.
  node_->tick_once(0.5);

  {
    std::lock_guard<std::mutex> lock(node_->map_mutex());
    const auto& prog = node_->map()[std::string(mowgli_map::layers::MOW_PROGRESS)];
    const float max_val = prog.maxCoeff();
    EXPECT_GT(max_val, 0.0F) << "Progress should not have fully decayed after 0.5 s";
    EXPECT_LT(max_val, 1.0F) << "Progress should have decreased";
    EXPECT_NEAR(max_val, 0.5F, 0.05F);
  }
}

TEST_F(MapServerTest, MowProgressDoesNotGoBelowZero)
{
  // Mark a cell mowed then decay far past zero
  node_->mark_mowed(0.0, 0.0);

  // Apply 10 seconds worth of decay (rate = 3600 / h → 10 units drop)
  node_->tick_once(10.0);

  {
    std::lock_guard<std::mutex> lock(node_->map_mutex());
    const auto& prog = node_->map()[std::string(mowgli_map::layers::MOW_PROGRESS)];
    EXPECT_FLOAT_EQ(prog.minCoeff(), 0.0F) << "mow_progress must be clamped at 0.0, not negative";
  }
}

TEST_F(MapServerTest, DecayRateParameterIsCorrect)
{
  EXPECT_DOUBLE_EQ(node_->decay_rate_per_hour(), 3600.0);
}

// ─────────────────────────────────────────────────────────────────────────────
// Test 3 — cell marking within mower width
// ─────────────────────────────────────────────────────────────────────────────

TEST_F(MapServerTest, MarkMowedSetsProgressToOne)
{
  node_->mark_mowed(0.0, 0.0);

  std::lock_guard<std::mutex> lock(node_->map_mutex());
  const auto& prog = node_->map()[std::string(mowgli_map::layers::MOW_PROGRESS)];
  // At least the centre cell should be 1.0
  EXPECT_FLOAT_EQ(prog.maxCoeff(), 1.0F);
}

TEST_F(MapServerTest, MarkMowedIncrementsConfidence)
{
  node_->mark_mowed(0.0, 0.0);

  std::lock_guard<std::mutex> lock(node_->map_mutex());
  const auto& conf = node_->map()[std::string(mowgli_map::layers::CONFIDENCE)];
  EXPECT_GT(conf.maxCoeff(), 0.0F) << "Confidence should be incremented when cells are mowed";
}

TEST_F(MapServerTest, OnlyMowerWidthRadiusIsMarked)
{
  // mower_width = 0.2 m → radius = 0.1 m.
  // A point at 0.5 m from centre should NOT be marked.
  node_->mark_mowed(0.0, 0.0);

  {
    std::lock_guard<std::mutex> lock(node_->map_mutex());
    const auto& m = node_->map();
    grid_map::Position far_pos(0.5, 0.5);
    grid_map::Index far_idx;
    ASSERT_TRUE(m.getIndex(far_pos, far_idx));
    const float far_val = m.at(std::string(mowgli_map::layers::MOW_PROGRESS), far_idx);
    EXPECT_FLOAT_EQ(far_val, 0.0F) << "Cell 0.5 m away should not be within mower_width radius";
  }
}

TEST_F(MapServerTest, MowerWidthParameterIsCorrect)
{
  EXPECT_DOUBLE_EQ(node_->mower_width(), 0.2);
}

// ─────────────────────────────────────────────────────────────────────────────
// Test 4 — classification enum values
// ─────────────────────────────────────────────────────────────────────────────

TEST_F(MapServerTest, CellTypeEnumValues)
{
  using mowgli_map::CellType;

  EXPECT_EQ(static_cast<uint8_t>(CellType::UNKNOWN), 0u);
  EXPECT_EQ(static_cast<uint8_t>(CellType::LAWN), 1u);
  EXPECT_EQ(static_cast<uint8_t>(CellType::OBSTACLE_PERMANENT), 2u);
  EXPECT_EQ(static_cast<uint8_t>(CellType::OBSTACLE_TEMPORARY), 3u);
  EXPECT_EQ(static_cast<uint8_t>(CellType::NO_GO_ZONE), 4u);
  EXPECT_EQ(static_cast<uint8_t>(CellType::DOCKING_AREA), 5u);
}

TEST_F(MapServerTest, CellTypeNamesAreCorrect)
{
  using mowgli_map::cell_type_name;
  using mowgli_map::CellType;

  EXPECT_EQ(cell_type_name(CellType::UNKNOWN), "UNKNOWN");
  EXPECT_EQ(cell_type_name(CellType::LAWN), "LAWN");
  EXPECT_EQ(cell_type_name(CellType::OBSTACLE_PERMANENT), "OBSTACLE_PERMANENT");
  EXPECT_EQ(cell_type_name(CellType::OBSTACLE_TEMPORARY), "OBSTACLE_TEMPORARY");
  EXPECT_EQ(cell_type_name(CellType::NO_GO_ZONE), "NO_GO_ZONE");
  EXPECT_EQ(cell_type_name(CellType::DOCKING_AREA), "DOCKING_AREA");
}

TEST_F(MapServerTest, ClassificationLayerDefaultIsUnknown)
{
  std::lock_guard<std::mutex> lock(node_->map_mutex());
  const auto& cls = node_->map()[std::string(mowgli_map::layers::CLASSIFICATION)];
  EXPECT_FLOAT_EQ(cls.minCoeff(), static_cast<float>(mowgli_map::CellType::UNKNOWN));
  EXPECT_FLOAT_EQ(cls.maxCoeff(), static_cast<float>(mowgli_map::CellType::UNKNOWN));
}

// ─────────────────────────────────────────────────────────────────────────────
// Test 5 — map clear resets all layers
// ─────────────────────────────────────────────────────────────────────────────

TEST_F(MapServerTest, ClearMapResetsAllLayersToDefault)
{
  // Dirty all layers
  node_->mark_mowed(0.0, 0.0);
  node_->tick_once(0.1);  // partially decay

  {
    std::lock_guard<std::mutex> lock(node_->map_mutex());
    auto& m = node_->map();
    // Manually set some cells in other layers
    grid_map::Index centre_idx;
    ASSERT_TRUE(m.getIndex(grid_map::Position(0.0, 0.0), centre_idx));
    m.at(std::string(mowgli_map::layers::OCCUPANCY), centre_idx) = 1.0F;
    m.at(std::string(mowgli_map::layers::CLASSIFICATION), centre_idx) =
        static_cast<float>(mowgli_map::CellType::NO_GO_ZONE);
  }

  // Now clear
  {
    std::lock_guard<std::mutex> lock(node_->map_mutex());
    node_->clear_map_layers();
  }

  // Verify all layers are back to defaults
  {
    std::lock_guard<std::mutex> lock(node_->map_mutex());
    const auto& m = node_->map();

    const auto& occ = m[std::string(mowgli_map::layers::OCCUPANCY)];
    const auto& cls = m[std::string(mowgli_map::layers::CLASSIFICATION)];
    const auto& prog = m[std::string(mowgli_map::layers::MOW_PROGRESS)];
    const auto& conf = m[std::string(mowgli_map::layers::CONFIDENCE)];

    EXPECT_FLOAT_EQ(occ.maxCoeff(), mowgli_map::defaults::OCCUPANCY);
    EXPECT_FLOAT_EQ(cls.maxCoeff(), mowgli_map::defaults::CLASSIFICATION);
    EXPECT_FLOAT_EQ(prog.maxCoeff(), mowgli_map::defaults::MOW_PROGRESS);
    EXPECT_FLOAT_EQ(conf.maxCoeff(), mowgli_map::defaults::CONFIDENCE);

    EXPECT_FLOAT_EQ(occ.minCoeff(), mowgli_map::defaults::OCCUPANCY);
    EXPECT_FLOAT_EQ(cls.minCoeff(), mowgli_map::defaults::CLASSIFICATION);
    EXPECT_FLOAT_EQ(prog.minCoeff(), mowgli_map::defaults::MOW_PROGRESS);
    EXPECT_FLOAT_EQ(conf.minCoeff(), mowgli_map::defaults::CONFIDENCE);
  }
}

TEST_F(MapServerTest, ClearMapDoesNotChangeGeometry)
{
  {
    std::lock_guard<std::mutex> lock(node_->map_mutex());
    node_->clear_map_layers();
    const auto& m = node_->map();
    EXPECT_EQ(m.getSize()(0), 100);
    EXPECT_EQ(m.getSize()(1), 100);
    EXPECT_DOUBLE_EQ(m.getResolution(), 0.1);
  }
}

// ─────────────────────────────────────────────────────────────────────────────
// Entry point
// ─────────────────────────────────────────────────────────────────────────────

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
