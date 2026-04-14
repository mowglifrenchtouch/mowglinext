package providers

import (
	"encoding/json"
	"fmt"
	"log"
	"sync"
	"time"

	"github.com/cedbossneo/mowglinext/pkg/types"
)

// MowingSessionRecord mirrors the API type for DB storage.
type MowingSessionRecord struct {
	ID              string   `json:"id"`
	StartTime       string   `json:"start_time"`
	EndTime         string   `json:"end_time"`
	DurationSec     float64  `json:"duration_sec"`
	AreaIndex       int      `json:"area_index"`
	CoveragePercent float32  `json:"coverage_percent"`
	StripsCompleted uint32   `json:"strips_completed"`
	StripsSkipped   uint32   `json:"strips_skipped"`
	DistanceMeters  float64  `json:"distance_meters"`
	Status          string   `json:"status"`
	Errors          []string `json:"errors"`
}

// SessionTracker monitors the BT high-level status and automatically
// records mowing sessions in the database.
type SessionTracker struct {
	dbProvider   types.IDBProvider
	mu           sync.Mutex
	currentState string
	sessionStart time.Time
	inSession    bool
}

const sessionsDBKey = "mowing.sessions"

// NewSessionTracker creates a tracker. Call Track() on each status update.
func NewSessionTracker(dbProvider types.IDBProvider) *SessionTracker {
	return &SessionTracker{
		dbProvider: dbProvider,
	}
}

// OnHighLevelStatus processes a raw JSON high-level status message.
// Call this from the fanOut pipeline for the "highLevelStatus" topic.
func (s *SessionTracker) OnHighLevelStatus(msg []byte) {
	var status struct {
		State     int     `json:"state"`
		StateName string  `json:"state_name"`
		Emergency bool    `json:"emergency"`
		Battery   float32 `json:"battery_percent"`
		Area      int     `json:"current_area"`
	}
	if err := json.Unmarshal(msg, &status); err != nil {
		return
	}

	s.mu.Lock()
	defer s.mu.Unlock()

	prevState := s.currentState
	s.currentState = status.StateName

	isMowing := status.State == 2 // HIGH_LEVEL_STATE_AUTONOMOUS
	wasMowing := prevState == "MOWING" || prevState == "TRANSIT" || prevState == "RECOVERING" || prevState == "RESUMING_AFTER_RAIN" || prevState == "RESUMING_UNDOCKING"

	// Start session
	if isMowing && !s.inSession {
		s.inSession = true
		s.sessionStart = time.Now().UTC()
		log.Printf("SessionTracker: mowing session started (state=%s)", status.StateName)
		return
	}

	// End session
	if s.inSession && !isMowing && !wasMowing {
		s.inSession = false
		endTime := time.Now().UTC()
		duration := endTime.Sub(s.sessionStart).Seconds()

		// Determine status
		sessionStatus := "completed"
		switch status.StateName {
		case "IDLE_DOCKED", "MOWING_COMPLETE":
			sessionStatus = "completed"
		case "COVERAGE_FAILED_DOCKING", "NAV_TO_DOCK_FAILED":
			sessionStatus = "error"
		default:
			if status.Emergency {
				sessionStatus = "error"
			} else {
				sessionStatus = "aborted"
			}
		}

		session := MowingSessionRecord{
			ID:          fmt.Sprintf("%d", s.sessionStart.UnixMilli()),
			StartTime:   s.sessionStart.Format(time.RFC3339),
			EndTime:     endTime.Format(time.RFC3339),
			DurationSec: duration,
			AreaIndex:   status.Area,
			Status:      sessionStatus,
			Errors:      []string{},
		}

		if status.Emergency {
			session.Errors = append(session.Errors, "Emergency stop triggered")
		}

		s.saveSession(session)
		log.Printf("SessionTracker: session ended (status=%s, duration=%.0fs)", sessionStatus, duration)
	}
}

func (s *SessionTracker) saveSession(session MowingSessionRecord) {
	// Load existing sessions
	sessions := []MowingSessionRecord{}
	if data, err := s.dbProvider.Get(sessionsDBKey); err == nil {
		_ = json.Unmarshal(data, &sessions)
	}

	sessions = append(sessions, session)

	// Keep last 500
	if len(sessions) > 500 {
		sessions = sessions[len(sessions)-500:]
	}

	data, err := json.Marshal(sessions)
	if err != nil {
		log.Printf("SessionTracker: failed to marshal sessions: %v", err)
		return
	}

	if err := s.dbProvider.Set(sessionsDBKey, data); err != nil {
		log.Printf("SessionTracker: failed to save session: %v", err)
	}
}
