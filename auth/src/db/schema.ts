/**
 * Database Schema with Drizzle ORM
 *
 * Defines the data models for:
 * - User (extended with hardware_bg and software_bg)
 * - Session (managed by better-auth)
 * - Account (OAuth/external auth, managed by better-auth)
 * - Verification Token (email verification, managed by better-auth)
 */

import { pgTable, text, timestamp, boolean, varchar, integer } from 'drizzle-orm/pg-core';
import { sql } from 'drizzle-orm';

/**
 * Users table with extended fields for the robotics platform
 *
 * Extends better-auth default user model with:
 * - hardware_bg: Hardware background (RTX4090, Jetson, Laptop, Cloud)
 * - software_bg: Software background (free text description)
 */
export const users = pgTable('user', {
  // Standard better-auth fields
  id: text('id').primaryKey(),
  name: text('name'),
  email: text('email').unique(),
  emailVerified: boolean('emailVerified').default(false),
  username: text('username').unique(),
  password: text('password'), // Hashed password

  // Custom fields for robotics platform
  hardware_bg: varchar('hardware_bg', { length: 20 }).notNull().default('Laptop'),
  software_bg: text('software_bg').default(''),
  skill_level: varchar('skill_level', { length: 20 }).notNull().default('Beginner'),

  // Metadata
  image: text('image'),
  createdAt: timestamp('createdAt').default(sql`CURRENT_TIMESTAMP`),
  updatedAt: timestamp('updatedAt').default(sql`CURRENT_TIMESTAMP`),
});

/**
 * Sessions table (managed by better-auth)
 * Stores active user sessions
 */
export const sessions = pgTable('session', {
  id: text('id').primaryKey(),
  expiresAt: timestamp('expiresAt').notNull(),
  token: text('token').unique().notNull(),
  createdAt: timestamp('createdAt').default(sql`CURRENT_TIMESTAMP`),
  updatedAt: timestamp('updatedAt').default(sql`CURRENT_TIMESTAMP`),
  ipAddress: text('ipAddress'),
  userAgent: text('userAgent'),
  userId: text('userId')
    .notNull()
    .references(() => users.id, { onDelete: 'cascade' }),
});

/**
 * Accounts table (managed by better-auth)
 * For OAuth and external authentication providers
 */
export const accounts = pgTable('account', {
  id: text('id').primaryKey(),
  userId: text('userId')
    .notNull()
    .references(() => users.id, { onDelete: 'cascade' }),
  accountId: text('accountId').notNull(),
  providerId: text('providerId').notNull(),
  accessToken: text('accessToken'),
  refreshToken: text('refreshToken'),
  idToken: text('idToken'),
  accessTokenExpiresAt: timestamp('accessTokenExpiresAt'),
  refreshTokenExpiresAt: timestamp('refreshTokenExpiresAt'),
  scope: text('scope'),
  password: text('password'),
  createdAt: timestamp('createdAt').default(sql`CURRENT_TIMESTAMP`),
  updatedAt: timestamp('updatedAt').default(sql`CURRENT_TIMESTAMP`),
});

/**
 * Verification tokens table (managed by better-auth)
 * For email verification, password reset, etc.
 */
export const verifications = pgTable('verification', {
  id: text('id').primaryKey(),
  identifier: text('identifier').notNull(),
  value: text('value').notNull(),
  expiresAt: timestamp('expiresAt').notNull(),
  createdAt: timestamp('createdAt').default(sql`CURRENT_TIMESTAMP`),
  updatedAt: timestamp('updatedAt').default(sql`CURRENT_TIMESTAMP`),
});

/**
 * Type exports for TypeScript
 */
export type User = typeof users.$inferSelect;
export type NewUser = typeof users.$inferInsert;
export type Session = typeof sessions.$inferSelect;
export type Account = typeof accounts.$inferSelect;
export type Verification = typeof verifications.$inferSelect;

/**
 * Hardware background validation
 * Valid values per spec: RTX4090, Jetson, Laptop, Cloud
 */
export const HARDWARE_BG_OPTIONS = ['RTX4090', 'Jetson', 'Laptop', 'Cloud'] as const;
export type HardwareBg = (typeof HARDWARE_BG_OPTIONS)[number];

/**
 * Validate hardware background value
 */
export function isValidHardwareBg(value: string): value is HardwareBg {
  return HARDWARE_BG_OPTIONS.includes(value as HardwareBg);
}

/**
 * Skill level validation
 * Valid values: Beginner, Advanced
 */
export const SKILL_LEVEL_OPTIONS = ['Beginner', 'Advanced'] as const;
export type SkillLevel = (typeof SKILL_LEVEL_OPTIONS)[number];

/**
 * Validate skill level value
 */
export function isValidSkillLevel(value: string): value is SkillLevel {
  return SKILL_LEVEL_OPTIONS.includes(value as SkillLevel);
}
