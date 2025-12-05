import type { VercelRequest, VercelResponse } from '@vercel/node';
import { db } from '../src/db/connection.js';
import { users } from '../src/db/simple-schema.js';
import { hashPassword, verifyPassword } from '../src/utils/password.js';
import { generateToken, verifyToken, extractTokenFromHeader } from '../src/utils/jwt.js';
import { eq } from 'drizzle-orm';

/**
 * Custom Authentication API
 *
 * Endpoints:
 * - POST /api/auth/signup - Create new user
 * - POST /api/auth/login - Login user
 * - GET /api/auth/me - Get current user
 */

// Enable CORS
function setCorsHeaders(res: VercelResponse, origin: string) {
  if (origin.includes('localhost') ||
      origin.endsWith('.vercel.app') ||
      origin.endsWith('.github.io')) {
    res.setHeader('Access-Control-Allow-Origin', origin);
    res.setHeader('Access-Control-Allow-Credentials', 'true');
  } else {
    res.setHeader('Access-Control-Allow-Origin', '*');
  }

  res.setHeader('Access-Control-Allow-Methods', 'GET, POST, PUT, DELETE, OPTIONS');
  res.setHeader('Access-Control-Allow-Headers', 'Content-Type, Authorization');
}

export default async function handler(req: VercelRequest, res: VercelResponse) {
  const origin = req.headers.origin || '';
  setCorsHeaders(res, origin);

  // Handle preflight
  if (req.method === 'OPTIONS') {
    res.status(200).end();
    return;
  }

  try {
    const pathname = req.url || '';
    console.log(`📥 Auth API request: ${req.method} ${pathname}`);

    // POST /api/auth/signup - Create new user
    if (pathname.includes('/signup') && req.method === 'POST') {
      const { email, password } = req.body;

      // Validate input
      if (!email || !password) {
        res.status(400).json({ error: 'Email and password are required' });
        return;
      }

      if (password.length < 8) {
        res.status(400).json({ error: 'Password must be at least 8 characters' });
        return;
      }

      // Check if user already exists
      const existingUsers = await db.select().from(users).where(eq(users.email, email));
      if (existingUsers.length > 0) {
        res.status(400).json({ error: 'User with this email already exists' });
        return;
      }

      // Hash password
      const password_hash = await hashPassword(password);

      // Create user
      const newUser = await db.insert(users).values({
        email,
        password_hash,
      }).returning({
        id: users.id,
        email: users.email,
        created_at: users.created_at,
      });

      // Generate JWT token
      const token = generateToken({
        userId: newUser[0].id,
        email: newUser[0].email,
      });

      console.log(`✅ User created: ${email}`);

      res.status(201).json({
        success: true,
        user: {
          id: newUser[0].id,
          email: newUser[0].email,
          created_at: newUser[0].created_at,
        },
        token,
      });
      return;
    }

    // POST /api/auth/login - Login user
    if (pathname.includes('/login') && req.method === 'POST') {
      const { email, password } = req.body;

      // Validate input
      if (!email || !password) {
        res.status(400).json({ error: 'Email and password are required' });
        return;
      }

      // Find user
      const foundUsers = await db.select().from(users).where(eq(users.email, email));
      if (foundUsers.length === 0) {
        res.status(401).json({ error: 'Invalid email or password' });
        return;
      }

      const user = foundUsers[0];

      // Verify password
      const isValid = await verifyPassword(password, user.password_hash);
      if (!isValid) {
        res.status(401).json({ error: 'Invalid email or password' });
        return;
      }

      // Generate JWT token
      const token = generateToken({
        userId: user.id,
        email: user.email,
      });

      console.log(`✅ User logged in: ${email}`);

      res.status(200).json({
        success: true,
        user: {
          id: user.id,
          email: user.email,
          created_at: user.created_at,
        },
        token,
      });
      return;
    }

    // GET /api/auth/me - Get current user
    if (pathname.includes('/me') && req.method === 'GET') {
      const token = extractTokenFromHeader(req.headers.authorization as string);
      if (!token) {
        res.status(401).json({ error: 'No token provided' });
        return;
      }

      const payload = verifyToken(token);
      if (!payload) {
        res.status(401).json({ error: 'Invalid or expired token' });
        return;
      }

      // Get user from database (include profile fields)
      const foundUsers = await db.select({
        id: users.id,
        email: users.email,
        hardware_bg: users.hardware_bg,
        skill_level: users.skill_level,
        created_at: users.created_at,
      }).from(users).where(eq(users.id, payload.userId));

      if (foundUsers.length === 0) {
        res.status(404).json({ error: 'User not found' });
        return;
      }

      res.status(200).json({
        success: true,
        user: foundUsers[0],
      });
      return;
    }

    // PUT /api/auth/profile - Update user profile
    if (pathname.includes('/profile') && req.method === 'PUT') {
      const token = extractTokenFromHeader(req.headers.authorization as string);
      if (!token) {
        res.status(401).json({ error: 'No token provided' });
        return;
      }

      const payload = verifyToken(token);
      if (!payload) {
        res.status(401).json({ error: 'Invalid or expired token' });
        return;
      }

      const { hardware_bg, skill_level } = req.body;

      // Validate input
      const validHardware = ['RTX4090', 'Jetson', 'Laptop', 'Cloud'];
      const validSkills = ['Beginner', 'Advanced'];

      if (hardware_bg && !validHardware.includes(hardware_bg)) {
        res.status(400).json({ error: 'Invalid hardware_bg value' });
        return;
      }

      if (skill_level && !validSkills.includes(skill_level)) {
        res.status(400).json({ error: 'Invalid skill_level value' });
        return;
      }

      // Update user profile
      const updateData: any = {
        updated_at: new Date(),
      };

      if (hardware_bg) updateData.hardware_bg = hardware_bg;
      if (skill_level) updateData.skill_level = skill_level;

      const updatedUsers = await db
        .update(users)
        .set(updateData)
        .where(eq(users.id, payload.userId))
        .returning({
          id: users.id,
          email: users.email,
          hardware_bg: users.hardware_bg,
          skill_level: users.skill_level,
          created_at: users.created_at,
        });

      console.log(`✅ Profile updated for user: ${payload.email}`);

      res.status(200).json({
        success: true,
        user: updatedUsers[0],
      });
      return;
    }

    // 404
    res.status(404).json({ error: 'Endpoint not found' });
  } catch (error) {
    console.error('❌ Error handling request:', error);
    res.status(500).json({
      error: 'Internal Server Error',
      message: error instanceof Error ? error.message : 'Unknown error',
    });
  }
}
