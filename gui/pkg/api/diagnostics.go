package api

import (
	"context"
	"net/http"
	"os"
	"strconv"
	"strings"
	"time"

	"github.com/cedbossneo/mowglinext/pkg/msgs/mowgli"
	"github.com/cedbossneo/mowglinext/pkg/types"
	"github.com/gin-gonic/gin"
)

// DiagnosticsSnapshot is the top-level response for GET /diagnostics/snapshot.
type DiagnosticsSnapshot struct {
	Timestamp  string             `json:"timestamp"`
	Containers []ContainerHealth  `json:"containers"`
	System     SystemHealth       `json:"system"`
	Coverage   []AreaCoverageInfo `json:"coverage"`
}

// ContainerHealth holds the health summary of a single Docker container.
type ContainerHealth struct {
	Name      string `json:"name"`
	State     string `json:"state"`
	Status    string `json:"status"`
	StartedAt string `json:"started_at"`
}

// SystemHealth holds host-level health metrics.
type SystemHealth struct {
	CpuTemperature float64 `json:"cpu_temperature"`
}

// AreaCoverageInfo holds per-area coverage data returned by the ROS service.
type AreaCoverageInfo struct {
	AreaIndex       uint32  `json:"area_index"`
	CoveragePercent float32 `json:"coverage_percent"`
	TotalCells      uint32  `json:"total_cells"`
	MowedCells      uint32  `json:"mowed_cells"`
	ObstacleCells   uint32  `json:"obstacle_cells"`
	StripsRemaining uint32  `json:"strips_remaining"`
}

// DiagnosticsRoutes registers the diagnostics endpoints on the provided router group.
func DiagnosticsRoutes(r *gin.RouterGroup, dockerProvider types.IDockerProvider, rosProvider types.IRosProvider) {
	group := r.Group("/diagnostics")
	group.GET("/snapshot", getDiagnosticsSnapshot(dockerProvider, rosProvider))
}

// getDiagnosticsSnapshot returns a diagnostic snapshot of the robot system.
//
// @Summary get diagnostics snapshot
// @Description returns container health, CPU temperature, and per-area coverage status
// @Tags diagnostics
// @Produce json
// @Success 200 {object} DiagnosticsSnapshot
// @Failure 500 {object} ErrorResponse
// @Router /diagnostics/snapshot [get]
func getDiagnosticsSnapshot(dockerProvider types.IDockerProvider, rosProvider types.IRosProvider) gin.HandlerFunc {
	return func(c *gin.Context) {
		ctx, cancel := context.WithTimeout(c.Request.Context(), 15*time.Second)
		defer cancel()

		snapshot := DiagnosticsSnapshot{
			Timestamp:  time.Now().UTC().Format(time.RFC3339),
			Containers: []ContainerHealth{},
			Coverage:   []AreaCoverageInfo{},
		}

		// --- Containers ---
		if containers, err := dockerProvider.ContainerList(ctx); err == nil {
			for _, ct := range containers {
				name := ""
				if len(ct.Names) > 0 {
					name = strings.TrimPrefix(ct.Names[0], "/")
				}
				startedAt := time.Unix(ct.Created, 0).UTC().Format(time.RFC3339)
				snapshot.Containers = append(snapshot.Containers, ContainerHealth{
					Name:      name,
					State:     ct.State,
					Status:    ct.Status,
					StartedAt: startedAt,
				})
			}
		}

		// --- System ---
		if data, err := os.ReadFile("/sys/class/thermal/thermal_zone0/temp"); err == nil {
			tempStr := strings.TrimSpace(string(data))
			if tempMilliC, err := strconv.ParseFloat(tempStr, 64); err == nil {
				snapshot.System.CpuTemperature = tempMilliC / 1000.0
			}
		}

		// --- Coverage (areas 0..19) ---
		for i := uint32(0); i < 20; i++ {
			req := mowgli.GetCoverageStatusReq{AreaIndex: i}
			var res mowgli.GetCoverageStatusRes
			if err := rosProvider.CallService(ctx, "/map_server_node/get_coverage_status", &req, &res, "mowgli_interfaces/srv/GetCoverageStatus"); err != nil {
				// ROS not reachable or area does not exist — stop iterating
				break
			}
			if !res.Success {
				// Area index does not exist; no more areas to query
				break
			}
			snapshot.Coverage = append(snapshot.Coverage, AreaCoverageInfo{
				AreaIndex:       i,
				CoveragePercent: res.CoveragePercent,
				TotalCells:      res.TotalCells,
				MowedCells:      res.MowedCells,
				ObstacleCells:   res.ObstacleCells,
				StripsRemaining: res.StripsRemaining,
			})
		}

		c.JSON(http.StatusOK, snapshot)
	}
}
