## Install system model

The installation system is currently in a transitional state.

### Configuration flow (intended design)

The intended installation workflow is:

1. `install/config/*` contains editable template files
2. The installer will:
   - read and modify these templates
   - generate a `.env`
   - copy and adapt files into `docker/config/*`
3. The Docker stack is launched using the generated `docker/config/*` and `.env`

Important:
- `install/config/*` is NOT the runtime configuration
- `docker/config/*` is the runtime configuration used by Docker Compose

### Current state

- The template → runtime copy/adaptation is NOT fully implemented yet
- Some configuration is still done manually
- The system must still be installable in its current form

### Review rules for install system

When reviewing installation and Docker-related code:

- Validate only the **current nominal install path**
- Do NOT treat the following as blocking issues:
  - differences between `install/config/*` and `docker/config/*`
  - unused or partially integrated template files
- Do NOT assume template processing is complete

Instead, focus on:
- whether the installer can run to completion
- whether `.env` contains required variables
- whether `docker compose config` is valid
- whether bind mounts point to existing runtime files
- whether the selected backend can start correctly

---

## Optional / future features (do not flag as critical)

The following are intentionally incomplete and should not be treated as blocking bugs:

- Foxglove activation is not yet automated (manual setup currently required)
- VESC support is not yet implemented (placeholders exist)
- TF-Luna / range sensor support is not yet implemented (placeholders exist)

These components:
- may appear in Docker Compose fragments
- may define variables not yet used
- should only be flagged if they break the default installation path

---

## Backend-specific install notes

- `HARDWARE_BACKEND` is the source of truth for runtime behavior
- All services that depend on backend selection should receive it explicitly
- Backend switching must not require manual patching of Compose files

Reviews must check:
- that backend-specific services receive the correct environment variables
- that enabling MAVROS does not leave services in inconsistent states

## Critical definition

A blocking issue is defined as:
- something that prevents the installer from completing
- something that prevents `docker compose config` from succeeding
- something that prevents the stack from starting

Everything else must be classified as:
- improvement
- future work
- or placeholder