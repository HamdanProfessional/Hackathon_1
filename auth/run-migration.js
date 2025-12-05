// Simple script to run SQL migration
import { neon } from '@neondatabase/serverless';
import { readFileSync } from 'fs';
import { join, dirname } from 'path';
import { fileURLToPath } from 'url';
import { config } from 'dotenv';

// ES module equivalent of __dirname
const __filename = fileURLToPath(import.meta.url);
const __dirname = dirname(__filename);

// Load .env from repository root
const envPath = join(__dirname, '../.env');
console.log('Loading .env from:', envPath);
config({ path: envPath, override: true });

async function runMigration() {
  if (!process.env.DATABASE_URL) {
    console.error('❌ DATABASE_URL not found in environment');
    process.exit(1);
  }

  // Fix DATABASE_URL if it has +asyncpg or other driver suffix
  let dbUrl = process.env.DATABASE_URL;
  if (dbUrl.includes('+asyncpg')) {
    dbUrl = dbUrl.replace('+asyncpg', '');
  }

  console.log('Database URL loaded:', dbUrl.substring(0, 30) + '...');

  const sql = neon(dbUrl);
  const migrationSQL = readFileSync(join(__dirname, 'create-users-table.sql'), 'utf-8');

  console.log('Running database migration...');

  try {
    // Split SQL into individual statements and execute them one by one
    const statements = migrationSQL
      .split(';')
      .map(s => s.trim())
      .filter(s => s.length > 0);

    console.log(`Executing ${statements.length} SQL statements...`);

    for (const statement of statements) {
      console.log(`Executing: ${statement.substring(0, 50)}...`);
      await sql(statement);
    }

    console.log('✅ Migration completed successfully!');
    console.log('Table created: users');
  } catch (error) {
    console.error('❌ Migration failed:', error);
    process.exit(1);
  }
}

runMigration();
