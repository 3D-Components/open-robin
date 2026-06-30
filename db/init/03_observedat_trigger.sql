-- Auto-populate observedAt from ts for DDS bridge data
-- 
-- NOTE: This script is provided as a reference. The 'attributes' table
-- is created by Orion-LD when it starts with TROE enabled, so this
-- trigger cannot be created at database init time.
--
-- To enable DDS temporal support, run AFTER docker compose up:
--   ./scripts/setup_dds_temporal.sh
--
-- The Orion-LD DDS bridge writes data without observedAt timestamps,
-- which breaks Mintaka temporal queries. The trigger below automatically
-- sets observedAt = ts when inserting new records.

-- Function to set observedAt from ts (can be created early)
CREATE OR REPLACE FUNCTION set_observedat_from_ts()
RETURNS TRIGGER AS $$
BEGIN
    IF NEW.observedat IS NULL THEN
        NEW.observedat := NEW.ts;
    END IF;
    RETURN NEW;
END;
$$ LANGUAGE plpgsql;

-- The trigger must be created after Orion-LD creates the attributes table:
-- 
-- DROP TRIGGER IF EXISTS trigger_set_observedat ON attributes;
-- CREATE TRIGGER trigger_set_observedat
--     BEFORE INSERT ON attributes
--     FOR EACH ROW
--     EXECUTE FUNCTION set_observedat_from_ts();
