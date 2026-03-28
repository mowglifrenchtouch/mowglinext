package api

import (
	"encoding/json"
	"fmt"
	"io"
	"log"
	"net/http"
	"sync"
	"time"

	"os"
	"path/filepath"
	"strconv"
	"strings"

	"github.com/cedbossneo/openmower-gui/pkg/types"
	"github.com/gin-gonic/gin"
	"github.com/joho/godotenv"
	"gopkg.in/yaml.v3"
)

const defaultSchemaURL = "https://raw.githubusercontent.com/ClemensElflein/OpenMowerMeta/main/backend/src/main/resources/assets/open_mower.default.schema.json"

func SettingsRoutes(r *gin.RouterGroup, dbProvider types.IDBProvider) {
	GetSettings(r, dbProvider)
	PostSettings(r, dbProvider)
	GetSettingsSchema(r, dbProvider)
	GetSettingsYAML(r, dbProvider)
	PostSettingsYAML(r, dbProvider)
}

func extractDefaults(schema map[string]any, defaults map[string]any) {
	if props, ok := schema["properties"].(map[string]any); ok {
		for key, prop := range props {
			if propMap, ok := prop.(map[string]any); ok {
				if def, hasDef := propMap["default"]; hasDef {
					defaults[key] = def
				}
				extractDefaults(propMap, defaults)
			}
		}
	}
	if allOf, ok := schema["allOf"].([]any); ok {
		for _, cond := range allOf {
			if condMap, ok := cond.(map[string]any); ok {
				if thenBlock, ok := condMap["then"].(map[string]any); ok {
					extractDefaults(thenBlock, defaults)
				}
				if elseBlock, ok := condMap["else"].(map[string]any); ok {
					extractDefaults(elseBlock, defaults)
				}
			}
		}
	}
}

// extractAllKeys collects every leaf property key defined in the schema,
// regardless of whether it has a default value. This is used to distinguish
// known schema properties from truly custom environment variables.
func extractAllKeys(schema map[string]any, keys map[string]bool) {
	if props, ok := schema["properties"].(map[string]any); ok {
		for key, prop := range props {
			if propMap, ok := prop.(map[string]any); ok {
				// Only mark leaf properties (those with a type that is not "object")
				// or properties with x-environment-variable as known keys.
				propType, _ := propMap["type"].(string)
				_, hasEnvVar := propMap["x-environment-variable"]
				if hasEnvVar || (propType != "" && propType != "object") {
					keys[key] = true
				}
				extractAllKeys(propMap, keys)
			}
		}
	}
	if allOf, ok := schema["allOf"].([]any); ok {
		for _, cond := range allOf {
			if condMap, ok := cond.(map[string]any); ok {
				if thenBlock, ok := condMap["then"].(map[string]any); ok {
					extractAllKeys(thenBlock, keys)
				}
				if elseBlock, ok := condMap["else"].(map[string]any); ok {
					extractAllKeys(elseBlock, keys)
				}
			}
		}
	}
}

// extractKeyTypes collects the JSON schema type for each leaf property key.
// This is used to coerce string values from mower_config.sh to the correct Go types.
func extractKeyTypes(schema map[string]any, types_ map[string]string) {
	if props, ok := schema["properties"].(map[string]any); ok {
		for key, prop := range props {
			if propMap, ok := prop.(map[string]any); ok {
				propType, _ := propMap["type"].(string)
				if propType != "" && propType != "object" {
					types_[key] = propType
				}
				extractKeyTypes(propMap, types_)
			}
		}
	}
	if allOf, ok := schema["allOf"].([]any); ok {
		for _, cond := range allOf {
			if condMap, ok := cond.(map[string]any); ok {
				if thenBlock, ok := condMap["then"].(map[string]any); ok {
					extractKeyTypes(thenBlock, types_)
				}
				if elseBlock, ok := condMap["else"].(map[string]any); ok {
					extractKeyTypes(elseBlock, types_)
				}
			}
		}
	}
}

// coerceValue converts a string value to the appropriate Go type based on the JSON schema type.
func coerceValue(value string, schemaType string) any {
	switch schemaType {
	case "boolean":
		return strings.EqualFold(value, "true")
	case "number":
		if f, err := strconv.ParseFloat(value, 64); err == nil {
			return f
		}
	case "integer":
		if i, err := strconv.ParseInt(value, 10, 64); err == nil {
			return i
		}
	}
	return value
}

func getSchema(dbProvider types.IDBProvider) (map[string]any, error) {
	schemaCacheMu.RLock()
	if schemaCache != nil && time.Since(schemaCacheTime) < schemaCacheTTL {
		cached := schemaCache
		schemaCacheMu.RUnlock()
		return cached, nil
	}
	schemaCacheMu.RUnlock()

	schemaURL := defaultSchemaURL
	if customURL, err := dbProvider.Get("system.mower.schemaURL"); err == nil && len(customURL) > 0 {
		schemaURL = string(customURL)
	}

	schema, err := fetchSchemaFromUpstream(schemaURL)
	if err != nil {
		localFile, localErr := os.ReadFile("asserts/mower_config.schema.json")
		if localErr != nil {
			return nil, fmt.Errorf("failed to fetch schema from upstream and no local fallback: %w", err)
		}
		if jsonErr := json.Unmarshal(localFile, &schema); jsonErr != nil {
			return nil, fmt.Errorf("invalid local schema JSON: %w", jsonErr)
		}
	}

	schema = applyMowgliOverlay(schema)

	schemaCacheMu.Lock()
	schemaCache = schema
	schemaCacheTime = time.Now()
	schemaCacheMu.Unlock()

	return schema, nil
}

// PostSettings saves the settings to the mower_config.sh file
//
// @Summary saves the settings to the mower_config.sh file
// @Description saves the settings to the mower_config.sh file
// @Tags settings
// @Accept  json
// @Produce  json
// @Param settings body map[string]any true "settings"
// @Success 200 {object} OkResponse
// @Failure 500 {object} ErrorResponse
// @Router /settings [post]
func PostSettings(r *gin.RouterGroup, dbProvider types.IDBProvider) gin.IRoutes {
	return r.POST("/settings", func(c *gin.Context) {
		var settingsPayload map[string]any
		err := c.BindJSON(&settingsPayload)
		if err != nil {
			c.JSON(500, ErrorResponse{
				Error: err.Error(),
			})
			return
		}
		mowerConfigFile, err := dbProvider.Get("system.mower.configFile")
		if err != nil {
			c.JSON(500, ErrorResponse{
				Error: err.Error(),
			})
			return
		}
		var settings = map[string]any{}
		configFileContent, err := os.ReadFile(string(mowerConfigFile))
		if err == nil {
			parse, err := godotenv.Parse(strings.NewReader(string(configFileContent)))
			if err == nil {
				for s, s2 := range parse {
					settings[s] = s2
				}
			}
		}

		// Merge defaults from schema
		schema, err := getSchema(dbProvider)
		defaults := map[string]any{}
		knownKeys := map[string]bool{}
		if err == nil {
			extractDefaults(schema, defaults)
			extractAllKeys(schema, knownKeys)
			for key, value := range defaults {
				if _, exists := settings[key]; !exists {
					settings[key] = value
				}
			}
		}

		// Identify which existing settings are custom (not in schema)
		customEnvVars := map[string]string{}
		for key, value := range settings {
			if !knownKeys[key] {
				if key != "custom_environment" {
					// We store them as strings, just in case
					customEnvVars[key] = fmt.Sprintf("%v", value)
				}
				delete(settings, key)
			}
		}

		// If frontend sends a new custom_environment, it completely replaces the old ones
		if customEnvObj, ok := settingsPayload["custom_environment"]; ok {
			// Clear existing custom env vars
			customEnvVars = map[string]string{}
			if customEnv, ok := customEnvObj.(map[string]any); ok {
				for k, v := range customEnv {
					customEnvVars[k] = fmt.Sprintf("%v", v)
				}
			}
			delete(settingsPayload, "custom_environment")
		}

		// Process the rest of the payload
		for key, value := range settingsPayload {
			settings[key] = value
		}

		// Re-inject custom env vars to be saved
		for k, v := range customEnvVars {
			settings[k] = v
		}

		// Write settings to file mower_config.sh
		var fileContent string
		for key, value := range settings {
			if key == "custom_environment" {
				continue
			}
			if value == true {
				value = "True"
			}
			if value == false {
				value = "False"
			}
			fileContent += "export " + key + "=" + fmt.Sprintf("%#v", value) + "\n"
		}
		if err = os.MkdirAll(filepath.Dir(string(mowerConfigFile)), 0755); err != nil {
			c.JSON(500, ErrorResponse{
				Error: err.Error(),
			})
			return
		}
		err = os.WriteFile(string(mowerConfigFile), []byte(fileContent), 0644)
		if err != nil {
			c.JSON(500, ErrorResponse{
				Error: err.Error(),
			})
			return
		}
		c.JSON(200, OkResponse{})
	})
}

// GetSettings returns a JSON object with the settings
//
// @Summary returns a JSON object with the settings
// @Description returns a JSON object with the settings
// @Tags settings
// @Produce  json
// @Success 200 {object} GetSettingsResponse
// @Failure 500 {object} ErrorResponse
// @Router /settings [get]
func GetSettings(r *gin.RouterGroup, dbProvider types.IDBProvider) gin.IRoutes {
	return r.GET("/settings", func(c *gin.Context) {
		mowerConfigFilePath, err := dbProvider.Get("system.mower.configFile")
		if err != nil {
			c.JSON(500, ErrorResponse{
				Error: err.Error(),
			})
			return
		}
		file, err := os.ReadFile(string(mowerConfigFilePath))
		if err != nil {
			if os.IsNotExist(err) {
				c.JSON(200, GetSettingsResponse{
					Settings: map[string]any{},
				})
				return
			}
			c.JSON(500, ErrorResponse{
				Error: err.Error(),
			})
			return
		}
		settings, err := godotenv.Parse(strings.NewReader(string(file)))
		if err != nil {
			c.JSON(500, ErrorResponse{
				Error: err.Error(),
			})
			return
		}
		schema, _ := getSchema(dbProvider)
		knownKeys := map[string]bool{}
		keyTypes := map[string]string{}
		if schema != nil {
			extractAllKeys(schema, knownKeys)
			extractKeyTypes(schema, keyTypes)
		}

		finalSettings := map[string]any{}
		customEnv := map[string]string{}
		for k, v := range settings {
			if knownKeys[k] || k == "custom_environment" {
				if t, ok := keyTypes[k]; ok {
					finalSettings[k] = coerceValue(v, t)
				} else {
					finalSettings[k] = v
				}
			} else {
				customEnv[k] = v
			}
		}

		if len(customEnv) > 0 {
			finalSettings["custom_environment"] = customEnv
		}

		c.JSON(200, GetSettingsResponse{
			Settings: finalSettings,
		})
	})
}

// syncToShellConfig writes OM_* keys from the payload into the mower_config.sh file.
// It reads the existing .sh file, merges the new OM_* values, and writes it back.
func syncToShellConfig(payload map[string]any, dbProvider types.IDBProvider) error {
	// Extract only OM_* keys from payload
	omKeys := map[string]any{}
	for key, value := range payload {
		if strings.HasPrefix(key, "OM_") {
			omKeys[key] = value
		}
	}
	if len(omKeys) == 0 {
		return nil
	}

	mowerConfigFile, err := dbProvider.Get("system.mower.configFile")
	if err != nil {
		return fmt.Errorf("get config file path: %w", err)
	}

	// Read existing shell config
	settings := map[string]any{}
	configFileContent, err := os.ReadFile(string(mowerConfigFile))
	if err == nil {
		parsed, parseErr := godotenv.Parse(strings.NewReader(string(configFileContent)))
		if parseErr == nil {
			for k, v := range parsed {
				settings[k] = v
			}
		}
	}

	// Merge defaults from schema
	schema, err := getSchema(dbProvider)
	if err == nil {
		defaults := map[string]any{}
		extractDefaults(schema, defaults)
		for key, value := range defaults {
			if _, exists := settings[key]; !exists {
				settings[key] = value
			}
		}
	}

	// Merge new OM_* values
	for key, value := range omKeys {
		settings[key] = value
	}

	// Write back
	var fileContent string
	for key, value := range settings {
		if value == true {
			value = "True"
		}
		if value == false {
			value = "False"
		}
		fileContent += "export " + key + "=" + fmt.Sprintf("%#v", value) + "\n"
	}

	if err := os.MkdirAll(filepath.Dir(string(mowerConfigFile)), 0755); err != nil {
		return fmt.Errorf("mkdir: %w", err)
	}
	if err := os.WriteFile(string(mowerConfigFile), []byte(fileContent), 0644); err != nil {
		return fmt.Errorf("write file: %w", err)
	}
	return nil
}

var (
	schemaCache     map[string]any
	schemaCacheMu   sync.RWMutex
	schemaCacheTime time.Time
	schemaCacheTTL  = 1 * time.Hour
)

// fetchSchemaFromUpstream fetches the JSON Schema from the upstream OpenMower repository.
func fetchSchemaFromUpstream(schemaURL string) (map[string]any, error) {
	client := &http.Client{Timeout: 15 * time.Second}
	resp, err := client.Get(schemaURL)
	if err != nil {
		return nil, fmt.Errorf("failed to fetch schema: %w", err)
	}
	defer resp.Body.Close()
	if resp.StatusCode != http.StatusOK {
		return nil, fmt.Errorf("upstream returned status %d", resp.StatusCode)
	}
	body, err := io.ReadAll(resp.Body)
	if err != nil {
		return nil, fmt.Errorf("failed to read schema response: %w", err)
	}
	var schema map[string]any
	if err := json.Unmarshal(body, &schema); err != nil {
		return nil, fmt.Errorf("invalid schema JSON from upstream: %w", err)
	}
	return schema, nil
}

// applyMowgliOverlay adds Mowgli-specific entries to the upstream schema:
// - Adds "Mowgli" to OM_MOWER enum
// - Adds conditional OM_NO_COMMS when Mowgli is selected
// - Hides OM_HARDWARE_VERSION and ESC_TYPE for Mowgli (they're not relevant)
// - Promotes GPS port/protocol out of "advanced" with Mowgli defaults
// - Sets Mowgli-specific defaults for battery voltages (YardForce 500B uses 24V)
func applyMowgliOverlay(schema map[string]any) map[string]any {
	props, ok := schema["properties"].(map[string]any)
	if !ok {
		return schema
	}
	hw, ok := props["important_settings"].(map[string]any)
	if !ok {
		return schema
	}
	hwProps, ok := hw["properties"].(map[string]any)
	if !ok {
		return schema
	}

	// Add "Mowgli" to OM_MOWER enum if not already present
	omMower, ok := hwProps["OM_MOWER"].(map[string]any)
	if ok {
		if enumList, ok := omMower["enum"].([]any); ok {
			hasMowgli := false
			for _, v := range enumList {
				if v == "Mowgli" {
					hasMowgli = true
					break
				}
			}
			if !hasMowgli {
				omMower["enum"] = append(enumList, "Mowgli")
			}
		}
	}

	// Collect non-Mowgli enum values for conditionals
	nonMowgliEnumValues := []any{}
	if omMower != nil {
		if enumList, ok := omMower["enum"].([]any); ok {
			for _, v := range enumList {
				if v != "Mowgli" {
					nonMowgliEnumValues = append(nonMowgliEnumValues, v)
				}
			}
		}
	}

	// Move OM_HARDWARE_VERSION and ESC_TYPE behind a conditional
	// so they only show for non-Mowgli builds
	hwVersion, hasHWVersion := hwProps["OM_HARDWARE_VERSION"]
	escType, hasESCType := hwProps["OM_MOWER_ESC_TYPE"]

	if hasHWVersion || hasESCType {
		delete(hwProps, "OM_HARDWARE_VERSION")
		delete(hwProps, "OM_MOWER_ESC_TYPE")

		nonMowgliProps := map[string]any{}
		if hasHWVersion {
			nonMowgliProps["OM_HARDWARE_VERSION"] = hwVersion
		}
		if hasESCType {
			nonMowgliProps["OM_MOWER_ESC_TYPE"] = escType
		}

		nonMowgliCondition := map[string]any{
			"if": map[string]any{
				"properties": map[string]any{
					"OM_MOWER": map[string]any{"enum": nonMowgliEnumValues},
				},
			},
			"then": map[string]any{
				"properties": nonMowgliProps,
			},
		}

		mowgliCondition := map[string]any{
			"if": map[string]any{
				"properties": map[string]any{
					"OM_MOWER": map[string]any{"const": "Mowgli"},
				},
			},
			"then": map[string]any{
				"properties": map[string]any{
					"OM_NO_COMMS": map[string]any{
						"type":                   "boolean",
						"default":                true,
						"title":                  "Disable OpenMower Comms",
						"description":            "Mowgli uses its own communication with the mainboard. This disables the OpenMower communication node.",
						"x-environment-variable": "OM_NO_COMMS",
					},
				},
			},
		}

		existingAllOf, _ := hw["allOf"].([]any)
		hw["allOf"] = append(existingAllOf, nonMowgliCondition, mowgliCondition)
	}

	// Promote GPS settings out of "advanced" toggle for Mowgli
	// and set Mowgli-specific defaults (GPS on /dev/gps, UBX protocol)
	applyMowgliGPSOverlay(props)

	// Promote mower logic advanced settings (includes OM_PERIMETER_SIGNAL
	// which is Mowgli-specific)
	promoteAdvancedSection(props, "mower_logic_settings", nil)

	return schema
}

// applyMowgliGPSOverlay promotes GPS settings from "advanced" and sets
// Mowgli-specific defaults.
func applyMowgliGPSOverlay(props map[string]any) {
	overrides := map[string]map[string]any{
		"OM_GPS_PORT": {
			"default":     "/dev/gps",
			"title":       "GPS Port",
			"description": "Serial port for the GPS board. Mowgli default: /dev/gps",
		},
		"OM_GPS_PROTOCOL": {
			"default": "UBX",
		},
		"OM_GPS_BAUDRATE": {
			"default": "460800",
			"title":   "GPS Baud Rate",
		},
	}
	promoteAdvancedSection(props, "gps_settings", overrides)
}

// promoteAdvancedSection removes the "advanced" boolean toggle from a schema
// section and moves all conditionally-shown properties into the base properties.
// Optional overrides let the caller set Mowgli-specific defaults on promoted fields.
func promoteAdvancedSection(props map[string]any, sectionKey string, overrides map[string]map[string]any) {
	section, ok := props[sectionKey].(map[string]any)
	if !ok {
		return
	}
	sectionProps, ok := section["properties"].(map[string]any)
	if !ok {
		return
	}
	allOf, ok := section["allOf"].([]any)
	if !ok {
		return
	}

	var advancedProps map[string]any
	var remainingAllOf []any

	for _, cond := range allOf {
		condMap, ok := cond.(map[string]any)
		if !ok {
			remainingAllOf = append(remainingAllOf, cond)
			continue
		}
		ifBlock, ok := condMap["if"].(map[string]any)
		if !ok {
			remainingAllOf = append(remainingAllOf, cond)
			continue
		}
		ifProps, ok := ifBlock["properties"].(map[string]any)
		if !ok {
			remainingAllOf = append(remainingAllOf, cond)
			continue
		}
		if _, isAdvanced := ifProps["advanced"]; isAdvanced {
			if thenBlock, ok := condMap["then"].(map[string]any); ok {
				if thenProps, ok := thenBlock["properties"].(map[string]any); ok {
					advancedProps = thenProps
				}
			}
			continue
		}
		remainingAllOf = append(remainingAllOf, cond)
	}

	if advancedProps == nil {
		return
	}

	// Remove the "advanced" toggle
	delete(sectionProps, "advanced")

	// Promote all advanced properties into base properties
	for key, prop := range advancedProps {
		if overrides != nil {
			if ov, ok := overrides[key]; ok {
				if propMap, ok := prop.(map[string]any); ok {
					for k, v := range ov {
						propMap[k] = v
					}
				}
			}
		}
		sectionProps[key] = prop
	}

	if len(remainingAllOf) > 0 {
		section["allOf"] = remainingAllOf
	} else {
		delete(section, "allOf")
	}
}

// GetSettingsSchema returns the JSON Schema describing all mower configuration parameters.
// It fetches the schema from the upstream OpenMower repository and caches it,
// then applies a Mowgli-specific overlay.
//
// @Summary returns the mower config JSON Schema
// @Description returns the JSON Schema for mower configuration parameters
// @Tags settings
// @Produce  json
// @Success 200 {object} map[string]any
// @Failure 500 {object} ErrorResponse
// @Router /settings/schema [get]
func GetSettingsSchema(r *gin.RouterGroup, dbProvider types.IDBProvider) gin.IRoutes {
	return r.GET("/settings/schema", func(c *gin.Context) {
		schema, err := getSchema(dbProvider)
		if err != nil {
			c.JSON(500, ErrorResponse{
				Error: err.Error(),
			})
			return
		}
		c.JSON(200, schema)
	})
}

// GetSettingsYAML returns the current YAML configuration values
//
// @Summary returns the current YAML mower configuration
// @Description returns the current YAML mower configuration values
// @Tags settings
// @Produce  json
// @Success 200 {object} map[string]any
// @Failure 500 {object} ErrorResponse
// @Router /settings/yaml [get]
func GetSettingsYAML(r *gin.RouterGroup, dbProvider types.IDBProvider) gin.IRoutes {
	return r.GET("/settings/yaml", func(c *gin.Context) {
		configFilePath, err := dbProvider.Get("system.mower.yamlConfigFile")
		if err != nil {
			c.JSON(500, ErrorResponse{
				Error: err.Error(),
			})
			return
		}
		file, err := os.ReadFile(string(configFilePath))
		if err != nil {
			// Return empty config if file doesn't exist yet
			if os.IsNotExist(err) {
				c.JSON(200, map[string]any{})
				return
			}
			c.JSON(500, ErrorResponse{
				Error: err.Error(),
			})
			return
		}
		var config map[string]any
		if err := yaml.Unmarshal(file, &config); err != nil {
			c.JSON(500, ErrorResponse{
				Error: "invalid YAML: " + err.Error(),
			})
			return
		}
		c.JSON(200, config)
	})
}

// PostSettingsYAML saves the mower configuration as YAML
//
// @Summary saves the mower configuration as YAML
// @Description saves the mower configuration as YAML, merging with existing values
// @Tags settings
// @Accept  json
// @Produce  json
// @Param settings body map[string]any true "settings"
// @Success 200 {object} OkResponse
// @Failure 500 {object} ErrorResponse
// @Router /settings/yaml [post]
func PostSettingsYAML(r *gin.RouterGroup, dbProvider types.IDBProvider) gin.IRoutes {
	return r.POST("/settings/yaml", func(c *gin.Context) {
		var payload map[string]any
		if err := c.BindJSON(&payload); err != nil {
			c.JSON(400, ErrorResponse{
				Error: err.Error(),
			})
			return
		}
		configFilePath, err := dbProvider.Get("system.mower.yamlConfigFile")
		if err != nil {
			c.JSON(500, ErrorResponse{
				Error: err.Error(),
			})
			return
		}
		// Read existing config and merge
		existing := map[string]any{}
		file, err := os.ReadFile(string(configFilePath))
		if err == nil {
			_ = yaml.Unmarshal(file, &existing)
		}

		// Merge defaults from schema
		schema, err := getSchema(dbProvider)
		if err == nil {
			defaults := map[string]any{}
			extractDefaults(schema, defaults)
			for key, value := range defaults {
				if _, exists := existing[key]; !exists {
					existing[key] = value
				}
			}
		}

		for key, value := range payload {
			existing[key] = value
		}
		out, err := yaml.Marshal(existing)
		if err != nil {
			c.JSON(500, ErrorResponse{
				Error: "failed to marshal YAML: " + err.Error(),
			})
			return
		}
		if err := os.MkdirAll(filepath.Dir(string(configFilePath)), 0755); err != nil {
			c.JSON(500, ErrorResponse{
				Error: err.Error(),
			})
			return
		}
		if err := os.WriteFile(string(configFilePath), out, 0644); err != nil {
			c.JSON(500, ErrorResponse{
				Error: err.Error(),
			})
			return
		}

		// Also sync OM_* keys to the .sh config file for backward compatibility
		if err := syncToShellConfig(payload, dbProvider); err != nil {
			log.Printf("Warning: failed to sync to shell config: %v", err)
		}

		c.JSON(200, OkResponse{})
	})
}
