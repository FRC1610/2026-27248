# Limelight pipelines

Backups of the Limelight 3A pipelines (`.vpr` files). No pipelines are stored here yet.

## Pipeline slots used by the code

| Slot | Pipeline | Constant |
| --- | --- | --- |
| 0 | Red Scoring Side | `PIPELINE_RED_SCORING_SIDE` |
| 1 | Red Audience Side | `PIPELINE_RED_AUDIENCE_SIDE` |
| 2 | Blue Audience Side | `PIPELINE_BLUE_AUDIENCE_SIDE` |
| 3 | Blue Scoring Side | `PIPELINE_BLUE_SCORING_SIDE` |
| 8 | All Red tags (red alliance default) | `PIPELINE_ALL_RED_TAGS` |
| 9 | All Blue tags (blue alliance default) | `PIPELINE_ALL_BLUE_TAGS` |

## Backing up

1. Connect to the Limelight web interface (`http://limelight.local:5801`, or through Panels).
2. Select the pipeline slot, then use **Download** (pipeline file) and save it here.
3. Name files by slot so they're easy to restore, e.g. `8_all_red_tags.vpr`.
4. Commit after any pipeline change (tag size, 3D settings, crop, exposure).

## Restoring

Select the slot in the web interface, use **Upload**, choose the matching `.vpr` file,
then save. Check the AprilTag size and Full 3D settings afterward: Main 2026's
target distance depends on them.
