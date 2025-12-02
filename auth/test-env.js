// Test environment loading
const { config } = require('dotenv');
const { resolve } = require('path');

console.log('BEFORE loading .env:');
console.log('DATABASE_URL:', process.env.DATABASE_URL);

// Load .env from parent directory WITH OVERRIDE
const result = config({ path: resolve(__dirname, '../.env'), override: true });

console.log('\nAFTER loading .env with override:');
console.log('Dotenv result:', result.error ? result.error.message : 'Success');
console.log('DATABASE_URL:', process.env.DATABASE_URL);
console.log('DATABASE_URL length:', process.env.DATABASE_URL?.length);
console.log('DATABASE_URL starts with:', process.env.DATABASE_URL?.substring(0, 50));
