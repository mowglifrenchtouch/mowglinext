package api

import (
	"os"
	"path/filepath"
	"testing"

	"github.com/cedbossneo/mowglinext/pkg/types"
	"github.com/stretchr/testify/assert"
	"github.com/stretchr/testify/require"
)

func TestBuildCrossChecks_IncludesGNSSFromEnv(t *testing.T) {
	t.Setenv("HARDWARE_BACKEND", "mowgli")
	t.Setenv("GNSS_BACKEND", "unicore")
	t.Setenv("GPS_CONNECTION", "usb")
	t.Setenv("GPS_PROTOCOL", "UNICORE")
	t.Setenv("GPS_PORT", "/dev/serial/by-id/usb-Unicore_UM980")
	t.Setenv("GPS_BY_ID", "/dev/serial/by-id/usb-Unicore_UM980")
	t.Setenv("GPS_BAUD", "460800")
	t.Setenv("GPS_FRAME_ID", "gps_antenna")

	tempDir := t.TempDir()
	yamlFile := filepath.Join(tempDir, "mowgli_robot.yaml")
	err := os.WriteFile(yamlFile, []byte(`mowgli:
  ros__parameters:
    datum_lat: 48.1234567
    datum_lon: 2.1234567
    dock_pose_x: 1.25
    dock_pose_y: -0.5
    dock_pose_yaw: 0.75
`), 0o644)
	require.NoError(t, err)

	db := types.NewMockDBProvider()
	require.NoError(t, db.Set("system.mower.yamlConfigFile", []byte(yamlFile)))

	checks := buildCrossChecks(db)

	assert.Equal(t, "unicore", checks.GNSS.Backend)
	assert.Equal(t, "mowgli", checks.GNSS.Hardware)
	assert.Equal(t, "usb", checks.GNSS.Connection)
	assert.Equal(t, "UNICORE", checks.GNSS.Protocol)
	assert.Equal(t, "/dev/serial/by-id/usb-Unicore_UM980", checks.GNSS.Port)
	assert.Equal(t, "/dev/serial/by-id/usb-Unicore_UM980", checks.GNSS.ByID)
	assert.Equal(t, "460800", checks.GNSS.Baud)
	assert.Equal(t, "gps_antenna", checks.GNSS.FrameID)
	assert.True(t, checks.GNSS.HasConfig)
	assert.Equal(t, "compose env", checks.GNSS.Source)
}
