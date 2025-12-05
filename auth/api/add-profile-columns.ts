import type { VercelRequest, VercelResponse } from '@vercel/node';
import { neon } from '@neondatabase/serverless';

/**
 * Add Profile Columns Migration
 *
 * Adds hardware_bg and skill_level columns to existing users table
 * Safe to run multiple times (uses IF NOT EXISTS)
 *
 * Usage: GET /api/add-profile-columns
 */

export default async function handler(req: VercelRequest, res: VercelResponse) {
  // Only allow GET requests
  if (req.method !== 'GET') {
    res.status(405).json({ error: 'Method not allowed' });
    return;
  }

  try {
    // Check if DATABASE_URL is set
    if (!process.env.DATABASE_URL) {
      res.status(500).json({ error: 'DATABASE_URL is not configured' });
      return;
    }

    const sql = neon(process.env.DATABASE_URL);

    console.log('Adding profile columns to users table...');

    // Add hardware_bg column if it doesn't exist
    const addHardwareBgSQL = `
      ALTER TABLE users
      ADD COLUMN IF NOT EXISTS hardware_bg TEXT;
    `;

    await sql(addHardwareBgSQL);
    console.log('✅ Added hardware_bg column');

    // Add skill_level column if it doesn't exist
    const addSkillLevelSQL = `
      ALTER TABLE users
      ADD COLUMN IF NOT EXISTS skill_level TEXT;
    `;

    await sql(addSkillLevelSQL);
    console.log('✅ Added skill_level column');

    res.status(200).json({
      success: true,
      message: 'Profile columns added successfully',
      columns: ['hardware_bg', 'skill_level'],
    });
  } catch (error) {
    console.error('Migration error:', error);
    res.status(500).json({
      error: 'Migration failed',
      message: error instanceof Error ? error.message : 'Unknown error',
    });
  }
}
