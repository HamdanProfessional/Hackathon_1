/**
 * Simplified Database Schema
 *
 * Minimal schema for authentication only
 * Custom fields can be added later via profile updates
 */

import { pgTable, text, timestamp, uuid } from 'drizzle-orm/pg-core';
import { sql } from 'drizzle-orm';

/**
 * Users table - authentication + optional profile fields
 */
export const users = pgTable('users', {
  id: uuid('id').primaryKey().defaultRandom(),
  email: text('email').notNull().unique(),
  password_hash: text('password_hash').notNull(),

  // Optional profile fields - can be updated after signup
  hardware_bg: text('hardware_bg'), // RTX4090, Jetson, Laptop, Cloud
  skill_level: text('skill_level'), // Beginner, Advanced

  created_at: timestamp('created_at').default(sql`CURRENT_TIMESTAMP`).notNull(),
  updated_at: timestamp('updated_at').default(sql`CURRENT_TIMESTAMP`).notNull(),
});

/**
 * Type exports for TypeScript
 */
export type User = typeof users.$inferSelect;
export type NewUser = typeof users.$inferInsert;
