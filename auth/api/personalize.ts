import type { VercelRequest, VercelResponse } from '@vercel/node';

/**
 * AI Content Personalization Endpoint
 *
 * Generates personalized educational content based on user's hardware and skill level
 * Uses OpenAI API to regenerate content tailored to the user's profile
 *
 * Usage: POST /api/personalize
 * Body: { pageContent: string, pageTitle: string, hardware_bg: string, skill_level: string }
 */

export default async function handler(req: VercelRequest, res: VercelResponse) {
  // Only allow POST requests
  if (req.method !== 'POST') {
    res.status(405).json({ error: 'Method not allowed' });
    return;
  }

  // Enable CORS
  res.setHeader('Access-Control-Allow-Origin', '*');
  res.setHeader('Access-Control-Allow-Methods', 'POST, OPTIONS');
  res.setHeader('Access-Control-Allow-Headers', 'Content-Type');

  // Handle preflight
  if (req.method === 'OPTIONS') {
    res.status(200).end();
    return;
  }

  try {
    const { pageContent, pageTitle, hardware_bg, skill_level } = req.body;

    // Validate input
    if (!pageContent || !hardware_bg || !skill_level) {
      res.status(400).json({ error: 'Missing required fields' });
      return;
    }

    console.log('🤖 Generating personalized content for:', {
      pageTitle,
      hardware: hardware_bg,
      skill: skill_level,
      contentLength: pageContent.length
    });

    // Check if OpenAI API key is configured
    if (!process.env.OPENAI_API_KEY) {
      console.warn('⚠️ OPENAI_API_KEY not configured, using mock response');

      // Return a mock personalized response for testing
      const mockContent = generateMockContent(pageTitle, hardware_bg, skill_level);
      res.status(200).json({ content: mockContent });
      return;
    }

    // Call OpenAI API to generate personalized content
    const openaiResponse = await fetch('https://api.openai.com/v1/chat/completions', {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
        'Authorization': `Bearer ${process.env.OPENAI_API_KEY}`,
      },
      body: JSON.stringify({
        model: 'gpt-4o-mini',
        messages: [
          {
            role: 'system',
            content: `You are an expert technical educator specializing in Physical AI, Robotics, and Machine Learning.
Your task is to personalize educational content based on the student's hardware and experience level.

Student Profile:
- Hardware: ${hardware_bg}
- Skill Level: ${skill_level}

Instructions:
1. Rewrite the content to be optimized for their specific hardware setup
2. Adjust complexity and explanations based on their skill level
3. Include hardware-specific code examples and commands where relevant
4. Add practical tips and warnings specific to their hardware
5. Use appropriate technical depth for their skill level
6. Format the response in clear, readable HTML with proper headings and code blocks

Keep the core learning objectives the same but tailor the delivery to their profile.`
          },
          {
            role: 'user',
            content: `Please personalize this educational content:\n\nTitle: ${pageTitle}\n\nContent:\n${pageContent.substring(0, 4000)}`
          }
        ],
        max_tokens: 2000,
        temperature: 0.7,
      }),
    });

    if (!openaiResponse.ok) {
      const errorData = await openaiResponse.json();
      console.error('OpenAI API error:', errorData);
      throw new Error('Failed to generate personalized content');
    }

    const data = await openaiResponse.json();
    const generatedContent = data.choices[0]?.message?.content || 'No content generated';

    console.log('✅ Content generated successfully');

    res.status(200).json({
      success: true,
      content: generatedContent,
    });
  } catch (error) {
    console.error('Personalization error:', error);
    res.status(500).json({
      error: 'Content generation failed',
      message: error instanceof Error ? error.message : 'Unknown error',
    });
  }
}

/**
 * Generate mock personalized content for testing (when OpenAI API key is not available)
 */
function generateMockContent(pageTitle: string, hardware_bg: string, skill_level: string): string {
  return `
    <div>
      <h2>🎯 Personalized Content: ${pageTitle}</h2>

      <div style="background: #e3f2fd; padding: 16px; border-radius: 8px; margin: 16px 0;">
        <h3>📊 Your Configuration</h3>
        <ul>
          <li><strong>Hardware:</strong> ${hardware_bg}</li>
          <li><strong>Skill Level:</strong> ${skill_level}</li>
        </ul>
      </div>

      <h3>🚀 Hardware-Specific Recommendations</h3>
      ${getHardwareRecommendations(hardware_bg)}

      <h3>📚 Skill-Level Tailored Content</h3>
      ${getSkillLevelContent(skill_level)}

      <div style="background: #fff3cd; padding: 16px; border-radius: 8px; margin: 16px 0;">
        <h4>💡 Pro Tip</h4>
        <p>This is a demo response. To get real AI-generated personalized content, configure the OPENAI_API_KEY environment variable in your Vercel deployment.</p>
      </div>
    </div>
  `;
}

function getHardwareRecommendations(hardware: string): string {
  const recommendations: Record<string, string> = {
    RTX4090: `
      <p>With your <strong>RTX 4090</strong>, you have access to cutting-edge AI capabilities:</p>
      <ul>
        <li>✅ Run full Isaac Sim simulations with ray-traced graphics</li>
        <li>✅ Train large-scale reinforcement learning models locally</li>
        <li>✅ Use high-resolution depth cameras and real-time SLAM</li>
        <li>⚡ Recommended batch size: 64-128 for model training</li>
      </ul>
      <pre><code># Optimize for RTX 4090
export CUDA_VISIBLE_DEVICES=0
export TF_FORCE_GPU_ALLOW_GROWTH=true</code></pre>
    `,
    Jetson: `
      <p>Your <strong>NVIDIA Jetson</strong> is perfect for edge deployment:</p>
      <ul>
        <li>✅ Focus on optimized inference rather than training</li>
        <li>✅ Use TensorRT for model acceleration</li>
        <li>✅ Leverage hardware-accelerated video encoding</li>
        <li>⚡ Recommended: Use INT8 quantization for better performance</li>
      </ul>
      <pre><code># Optimize for Jetson
sudo nvpmodel -m 0  # Max performance mode
sudo jetson_clocks    # Max clock speeds</code></pre>
    `,
    Laptop: `
      <p>Working with <strong>CPU-only</strong> setup requires smart optimizations:</p>
      <ul>
        <li>✅ Use lightweight simulation tools (Gazebo instead of Isaac Sim)</li>
        <li>✅ Focus on pre-trained models and transfer learning</li>
        <li>✅ Consider cloud resources for heavy computation</li>
        <li>⚡ Recommended: Use Colab for training, local for development</li>
      </ul>
      <pre><code># CPU optimization
export OMP_NUM_THREADS=4
export MKL_NUM_THREADS=4</code></pre>
    `,
    Cloud: `
      <p>Using <strong>Cloud/Colab</strong> gives you flexible resources:</p>
      <ul>
        <li>✅ Scale GPU resources as needed for training</li>
        <li>✅ Use persistent storage for datasets</li>
        <li>✅ Leverage prebuilt environments and notebooks</li>
        <li>⚡ Recommended: Save models frequently to avoid session timeouts</li>
      </ul>
      <pre><code># Mount Google Drive in Colab
from google.colab import drive
drive.mount('/content/drive')</code></pre>
    `,
  };

  return recommendations[hardware] || '<p>Hardware-specific recommendations not available.</p>';
}

function getSkillLevelContent(skill: string): string {
  const content: Record<string, string> = {
    Beginner: `
      <p>As a <strong>beginner</strong>, let's break this down step by step:</p>
      <ol>
        <li><strong>Start with basics:</strong> Understand what each component does before diving into code</li>
        <li><strong>Follow examples:</strong> Copy and run provided code to see how it works</li>
        <li><strong>Experiment safely:</strong> Use simulation before trying on real hardware</li>
        <li><strong>Ask questions:</strong> No question is too basic - robotics is complex!</li>
      </ol>
      <div style="background: #e8f5e9; padding: 12px; border-radius: 6px;">
        <p><strong>🎓 Learning Path:</strong> Theory → Simple Examples → Hands-on Practice → Build Projects</p>
      </div>
    `,
    Advanced: `
      <p>As an <strong>advanced</strong> learner, you can dive deeper:</p>
      <ul>
        <li><strong>Optimize performance:</strong> Profile code, tune hyperparameters, benchmark alternatives</li>
        <li><strong>Customize architectures:</strong> Modify provided code to fit your specific needs</li>
        <li><strong>Integrate multiple systems:</strong> Combine perception, planning, and control</li>
        <li><strong>Contribute back:</strong> Share your optimizations and discoveries</li>
      </ul>
      <div style="background: #e8f5e9; padding: 12px; border-radius: 6px;">
        <p><strong>💡 Challenge:</strong> Try implementing the concepts using different frameworks or approaches</p>
      </div>
    `,
  };

  return content[skill] || '<p>Skill-level content not available.</p>';
}
