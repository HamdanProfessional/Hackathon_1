// Simple script to run SQL migration
const { neon } = require('@neondatabase/serverless');
const fs = require('fs');
const path = require('path');

// Load .env from repository root
const envPath = path.join(__dirname, '../.env');
console.log('Loading .env from:', envPath);
require('dotenv').config({ path: envPath, override: true });

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
  const migrationSQL = fs.readFileSync(path.join(__dirname, 'migrate.sql'), 'utf-8');

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
    console.log('Tables created: user, session, account, verification');
  } catch (error) {
    console.error('❌ Migration failed:', error);
    process.exit(1);
  }
}

runMigration();
