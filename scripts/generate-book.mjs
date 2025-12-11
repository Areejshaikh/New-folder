import { GoogleAuth } from 'google-auth-library';
import fetch from 'node-fetch';
import fs from 'fs';
import path from 'path';

// This script will generate the Docusaurus book content.

// TODO: Replace with the actual path to your Google Cloud service account key file
const KEY_FILE_PATH = 'path/to/your/service-account-key.json';
const MODEL_ID = 'gemini-pro';
const PROJECT_ID = 'your-gcp-project-id'; // TODO: Replace with your GCP project ID

async function callGeminiApi(prompt) {
    console.log('Authenticating with Google Cloud...');
    const auth = new GoogleAuth({
        keyFilename: KEY_FILE_PATH,
        scopes: 'https://www.googleapis.com/auth/cloud-platform',
    });

    try {
        const client = await auth.getClient();
        const accessToken = (await client.getAccessToken()).token;
        console.log('Successfully authenticated.');

        const endpoint = `https://us-central1-aiplatform.googleapis.com/v1/projects/${PROJECT_ID}/locations/us-central1/publishers/google/models/${MODEL_ID}:streamGenerateContent`;

        console.log('Calling Gemini API...');
        const response = await fetch(endpoint, {
            method: 'POST',
            headers: {
                'Authorization': `Bearer ${accessToken}`,
                'Content-Type': 'application/json',
            },
            body: JSON.stringify({
                contents: [{
                    parts: [{
                        text: prompt
                    }]
                }]
            }),
        });

        if (!response.ok) {
            throw new Error(`API call failed with status: ${response.status} ${response.statusText}`);
        }

        const responseBody = await response.json();
        // Assuming the response is an array of content parts
        const generatedText = responseBody.map(item => item.candidates[0].content.parts[0].text).join('');

        console.log('Successfully received response from Gemini API.');
        return generatedText;

    } catch (error) {
        console.error('Error calling Gemini API:', error.message);
        // Return a placeholder or throw to stop execution
        return `[Error generating content for this section: ${error.message}]`;
    }
}


function parseArgs() {
    const args = process.argv.slice(2);
    const inputArg = args.find(arg => arg.startsWith('--input='));
    if (!inputArg) {
        console.error('Error: Missing required --input argument. Usage: node scripts/generate-book.mjs --input=<path_to_config.json>');
        process.exit(1);
    }
    return {
        inputPath: inputArg.split('=')[1],
    };
}

function generateSectionPrompt(chapterTitle, sectionType) {
    return `You are an expert in robotics and AI, writing a chapter for a book titled "Physical AI & Humanoid Robotics".
The chapter you are writing is: "${chapterTitle}".
Generate the content for the "${sectionType}" section of this chapter.

Follow these rules:
- Use simple English, suitable for advanced students.
- Do not add any extra explanation or titles, only the clean content for the section.
- For "Code examples" sections, provide clear, runnable Python/ROS2 code blocks.
- For "Diagrams" sections, create a clear ASCII diagram to illustrate the concepts.
- For "Student exercises" sections, provide 2-3 practical exercises.

Generate only the content for the "${sectionType}" section.`;
}

function assembleChapterContent(chapterTitle, chapterContent, sections) {
    let fullContent = `# ${chapterTitle}\n\n`;
    for (const section of sections) {
        fullContent += `## ${section}\n\n`;
        fullContent += `${chapterContent[section] || ''}\n\n`;
    }
    return fullContent;
}

function titleToKebabCase(title) {
    return title.toLowerCase()
        .replace(/[^a-z0-9\s-]/g, '') // remove special characters
        .replace(/\s+/g, '-')         // replace spaces with hyphens
        .replace(/-+/g, '-');          // remove consecutive hyphens
}

async function main() {
    const { inputPath } = parseArgs();
    console.log(`Input file path: ${inputPath}`);

    let bookConfig;
    try {
        const fileContent = fs.readFileSync(inputPath, 'utf8');
        bookConfig = JSON.parse(fileContent);
    } catch (error) {
        if (error.code === 'ENOENT') {
            console.error(`Error: Input file not found at '${inputPath}'`);
        } else if (error instanceof SyntaxError) {
            console.error(`Error: Invalid JSON in input file '${inputPath}'. ${error.message}`);
        } else {
            console.error(`An unexpected error occurred: ${error.message}`);
        }
        process.exit(1);
    }

    console.log('Successfully parsed book configuration.');

    const sections = ["Overview", "Key Concepts", "Step-by-step explanations", "Code examples", "Diagrams", "Summary", "Student exercises"];
    const outputDir = 'docusaurus-book/docs';
    let chapterCounter = 1;

    if (!fs.existsSync(outputDir)) {
        fs.mkdirSync(outputDir, { recursive: true });
    }

    for (const module of bookConfig.modules) {
        console.log(`Processing module: ${module.title}`);
        for (const chapter of module.chapters) {
            console.log(`-- Processing chapter: ${chapter}`);
            const chapterContent = {};
            for (const section of sections) {
                const prompt = generateSectionPrompt(chapter, section);
                console.log(`---- Generating content for section: ${section}`);
                chapterContent[section] = await callGeminiApi(prompt);
            }

            const assembledContent = assembleChapterContent(chapter, chapterContent, sections);

            const fileName = `${chapterCounter}-${titleToKebabCase(chapter)}.md`;
            const filePath = path.join(outputDir, fileName);

            try {
                fs.writeFileSync(filePath, assembledContent, 'utf8');
                console.log(`Successfully wrote chapter to ${filePath}`);
            } catch (error) {
                console.error(`Error writing file ${filePath}: ${error.message}`);
            }

            chapterCounter++;
        }
    }
}

main();

