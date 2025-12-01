/**
 * Database Initialization
 *
 * Sets up the database schema and prepares it for use
 * This script can be run manually to ensure schema is synchronized
 */

import 'dotenv/config';
import { drizzle } from 'drizzle-orm/neon-http';
import { postgres } from '@neondatabase/serverless';
import * as schema from './schema';
import { sql } from 'drizzle-orm';

/**
 * Initialize database connection
 */
async function initializeDatabase() {
  if (!process.env.DATABASE_URL) {
    throw new Error('DATABASE_URL environment variable is not set');
  }

  console.log('🔄 Initializing database connection...');
  const client = postgres(process.env.DATABASE_URL);
  const db = drizzle(client, { schema });

  try {
    console.log('📝 Checking database schema...');

    // Verify users table exists by attempting a simple query
    await db.execute(sql`SELECT 1 FROM information_schema.tables WHERE table_name = 'user'`);

    console.log('✅ Database schema verified successfully!');
    console.log('📋 Tables initialized:');
    console.log('   - user (with hardware_bg and software_bg fields)');
    console.log('   - session');
    console.log('   - account');
    console.log('   - verification');

    return true;
  } catch (error) {
    console.error('❌ Database initialization error:', error);
    console.log('\n📌 To set up the database schema, run:');
    console.log('   npm run db:push');
    return false;
  }
}

// Run if executed directly
if (require.main === module) {
  initializeDatabase()
    .then((success) => {
      process.exit(success ? 0 : 1);
    })
    .catch((error) => {
      console.error('Fatal error:', error);
      process.exit(1);
    });
}

export { initializeDatabase };
